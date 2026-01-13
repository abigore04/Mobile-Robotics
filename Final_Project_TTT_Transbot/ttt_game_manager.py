#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from ttt_vision.srv import GotoCell

import random
import math
import time

# preset storage coordinates (row,col)
STORAGE_X = [(0, 0), (0, 1), (0, 2), (1, 0), (1, 1)]
STORAGE_O = [(1, 2), (2, 0), (2, 1), (2, 2)]

# optional celebration services that can be launched after run of ttt_celebration_node.py to enable)
CELEBRATE_WIN  = "/ttt/celebrate/win"
CELEBRATE_LOSE = "/ttt/celebrate/lose"
CELEBRATE_DRAW = "/ttt/celebrate/draw"


def parse_board_string(s):
    rows = s.strip().split("/")
    if len(rows) != 3:
        return None
    mat = [list(r) for r in rows]
    if any(len(r) != 3 for r in mat):
        return None
    for r in range(3):
        for c in range(3):
            if mat[r][c] not in ["X", "O", "."]:
                return None
    return mat


def print_board(mat, header=None):
    if header:
        print("\n" + header)
    for r in range(3):
        print(mat[r][0], mat[r][1], mat[r][2])
    print("")


# all possible combination to win = 8
WIN_LINES = [
    [(0, 0), (0, 1), (0, 2)],
    [(1, 0), (1, 1), (1, 2)],
    [(2, 0), (2, 1), (2, 2)],
    [(0, 0), (1, 0), (2, 0)],
    [(0, 1), (1, 1), (2, 1)],
    [(0, 2), (1, 2), (2, 2)],
    [(0, 0), (1, 1), (2, 2)],
    [(0, 2), (1, 1), (2, 0)],
]


def winner(b):
    for line in WIN_LINES:
        a, b1, c = line
        v1 = b[a[0]][a[1]]
        v2 = b[b1[0]][b1[1]]
        v3 = b[c[0]][c[1]]
        if v1 != "." and v1 == v2 == v3:
            return v1
    return None


def is_full(b):
    return all(b[r][c] != "." for r in range(3) for c in range(3))


def legal_moves(b):
    return [(r, c) for r in range(3) for c in range(3) if b[r][c] == "."]


def terminal_value(b, AI, HUMAN):
    w = winner(b)
    if w == AI:
        return 1
    if w == HUMAN:
        return -1
    if is_full(b):
        return 0
    return None


def minimax(b, maximizing, AI, HUMAN):
    tv = terminal_value(b, AI, HUMAN)
    if tv is not None:
        return tv, None

    if maximizing:
        best = -math.inf
        best_move = None
        for (r, c) in legal_moves(b):
            b[r][c] = AI
            score, _ = minimax(b, False, AI, HUMAN)
            b[r][c] = "."
            if score > best:
                best = score
                best_move = (r, c)
        return best, best_move
    else:
        best = math.inf
        best_move = None
        for (r, c) in legal_moves(b):
            b[r][c] = HUMAN
            score, _ = minimax(b, True, AI, HUMAN)
            b[r][c] = "."
            if score < best:
                best = score
                best_move = (r, c)
        return best, best_move


class GameManager:
    def __init__(self):
        self.latest_board = None
        self.latest_board_str = None
        self.have_board = False

        rospy.Subscriber("/ttt/board_matrix", String, self.cb_board)

        rospy.wait_for_service("/ttt/arm/go_home")
        rospy.wait_for_service("/ttt/arm/goto_storage")
        rospy.wait_for_service("/ttt/arm/goto_playground")

        self.srv_home = rospy.ServiceProxy("/ttt/arm/go_home", Trigger)
        self.srv_goto_storage = rospy.ServiceProxy("/ttt/arm/goto_storage", GotoCell)
        self.srv_goto_playground = rospy.ServiceProxy("/ttt/arm/goto_playground", GotoCell)

        # Optional (do NOT wait — if missing, game still runs)
        self.srv_celebrate_win = None
        self.srv_celebrate_lose = None
        self.srv_celebrate_draw = None

        self.reset_storage_lists()

    def reset_storage_lists(self):
        self.available_X = STORAGE_X.copy()
        self.available_O = STORAGE_O.copy()

    def cb_board(self, msg):
        mat = parse_board_string(msg.data)
        if mat is None:
            return
        self.latest_board = mat
        self.latest_board_str = msg.data
        self.have_board = True

    def wait_for_board(self, timeout=20.0):
        t0 = time.time()
        while not rospy.is_shutdown() and not self.have_board:
            if time.time() - t0 > timeout:
                return False
            time.sleep(0.05)
        return True

    def _ensure_celebrate(self):
        if self.srv_celebrate_win is not None:
            return
        try:
            self.srv_celebrate_win = rospy.ServiceProxy(CELEBRATE_WIN, Trigger)
            self.srv_celebrate_lose = rospy.ServiceProxy(CELEBRATE_LOSE, Trigger)
            self.srv_celebrate_draw = rospy.ServiceProxy(CELEBRATE_DRAW, Trigger)
        except Exception:
            self.srv_celebrate_win = None
            self.srv_celebrate_lose = None
            self.srv_celebrate_draw = None

    def celebrate(self, outcome):
        """
        outcome: "ROBOT_WIN" / "HUMAN_WIN" / "DRAW"
        Calls optional celebration services if available.
        """
        self._ensure_celebrate()
        try:
            if outcome == "ROBOT_WIN" and self.srv_celebrate_win:
                self.srv_celebrate_win()
            elif outcome == "HUMAN_WIN" and self.srv_celebrate_lose:
                self.srv_celebrate_lose()
            elif outcome == "DRAW" and self.srv_celebrate_draw:
                self.srv_celebrate_draw()
        except Exception as e:
            print("celebration failed:", e)

    def place_piece(self, symbol, target_r, target_c):
        if symbol == "X":
            if not self.available_X:
                print("ERROR: no X pieces left in storage list!")
                return False
            sr, sc = self.available_X.pop(0)
        else:
            if not self.available_O:
                print("ERROR: no O pieces left in storage list!")
                return False
            sr, sc = self.available_O.pop(0)

        print(f"Robot placing {symbol}: storage ({sr},{sc}) -> playground ({target_r},{target_c})")

        resp = self.srv_goto_storage(sr, sc)
        if not resp.ok:
            print("goto_storage failed:", resp.msg)
            return False

        resp = self.srv_goto_playground(target_r, target_c)
        if not resp.ok:
            print("goto_playground failed:", resp.msg)
            return False

        h = self.srv_home()
        if not h.success:
            print("go_home failed:", h.message)
            return False

        return True

    def interactive_mode(self):
        self.reset_storage_lists()

        while True:
            choice = input("Choose: [A]=Human X (robot O), [P]=Human O (robot X): ").strip().upper()
            if choice in ["A", "P"]:
                break

        if choice == "A":
            HUMAN, ROBOT = "X", "O"
        else:
            HUMAN, ROBOT = "O", "X"

        print(f"\nInteractive: HUMAN={HUMAN}, ROBOT={ROBOT}")
        print("Waiting for vision board...")

        if not self.wait_for_board():
            print("ERROR: no /ttt/board_matrix received.")
            return

        print_board(self.latest_board, header="[START] Board:")

        turn = "X"
        step = 1
        last_seen = self.latest_board_str

        while True:
            b = self.latest_board
            w = winner(b)
            if w or is_full(b):
                break

            print(f"Step {step}  Turn={turn}")

            if turn == HUMAN:
                print("Waiting for human move (place piece by hand)...")
                while not rospy.is_shutdown() and self.latest_board_str == last_seen:
                    time.sleep(0.05)
                last_seen = self.latest_board_str
                print_board(self.latest_board, header="[HUMAN MOVE] Board:")
            else:
                score, move = minimax([row[:] for row in b], True, ROBOT, HUMAN)
                if move is None:
                    break
                r, c = move
                print(f"Robot chooses ({r},{c}) minimax_score={score}")
                ok = self.place_piece(ROBOT, r, c)
                if not ok:
                    print("Robot placement failed.")
                    return
                print("Waiting vision confirmation (board must change)...")
                while not rospy.is_shutdown() and self.latest_board_str == last_seen:
                    time.sleep(0.05)
                last_seen = self.latest_board_str
                print_board(self.latest_board, header="[AFTER ROBOT] Board:")

            turn = "O" if turn == "X" else "X"
            step += 1

        final = winner(self.latest_board)
        if final is None:
            print("GAME OVER: DRAW")
            self.celebrate("DRAW")
        elif final == HUMAN:
            print("GAME OVER: HUMAN WIN")
            self.celebrate("HUMAN_WIN")
        else:
            print("GAME OVER: ROBOT WIN")
            self.celebrate("ROBOT_WIN")

    def autonomous_mode(self):
        self.reset_storage_lists()

        print("\nAutonomous: random moves for both sides.")
        print("Waiting for vision board...")

        if not self.wait_for_board():
            print("ERROR: no /ttt/board_matrix received.")
            return

        print_board(self.latest_board, header="[START] Board:")

        turn = "X"
        step = 1
        last_seen = self.latest_board_str

        while True:
            b = self.latest_board
            w = winner(b)
            if w or is_full(b):
                break

            moves = legal_moves(b)
            if not moves:
                break

            r, c = random.choice(moves)
            print(f"Step {step}  Turn={turn}  move=({r},{c})")

            ok = self.place_piece(turn, r, c)
            if not ok:
                print("Placement failed.")
                return

            print("Waiting vision confirmation (board must change)...")
            while not rospy.is_shutdown() and self.latest_board_str == last_seen:
                time.sleep(0.05)
            last_seen = self.latest_board_str

            print_board(self.latest_board, header="[UPDATED] Board:")
            turn = "O" if turn == "X" else "X"
            step += 1

        final = winner(self.latest_board)
        if final is None:
            print("GAME OVER: DRAW")
            self.celebrate("DRAW")
        else:
            print("GAME OVER: WINNER =", final)
            # In autonomous, just celebrate as a “win”
            self.celebrate("ROBOT_WIN")


def main():
    rospy.init_node("ttt_game_manager", anonymous=True)
    gm = GameManager()

    print("\n=== TTT GAME MANAGER (TERMINAL) ===")
    print("1) Interactive (Human vs Robot, minimax)")
    print("2) Autonomous (Robot vs Robot random)")
    mode = input("Choose mode [1/2]: ").strip()

    if mode == "1":
        gm.interactive_mode()
    else:
        gm.autonomous_mode()


if __name__ == "__main__":
    main()
