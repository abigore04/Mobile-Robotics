#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String

import threading
import sys
import time

bridge = CvBridge()
# Camera topic
CAMERA_TOPIC ="/camera/rgb/image_raw"

# Board warp size
WARP_SIZE =300  # pixels (board will be WARP_SIZE x WARP_SIZE)

# Crop inside each extracted cell to avoid borders/grid
CELL_CROP_MODE= "frac"   # "frac" or "px"
CELL_MARGIN_FRAC =0.50
CELL_MARGIN_PX= 8


# V thresholds (your observation)
#   Empty:140..200
#   O: 220..255
#   X: 0..130
V_X_MAX = 130
V_EMPTY_MIN = 140
V_EMPTY_MAX = 200
V_O_MIN = 220

# Timing
PERIODIC_SEC = 5.0
CHANGE_DEBOUNCE_SEC = 0.50
BOARD_SEEN_TIMEOUT_SEC = 2.0

# Buzzer settings
BEEP_ON_TIME_MS = 100
BEEP_GAP_SEC = 0.12

# Your mapping: window cells -> matrix row/col
WIN_CELL_TO_MATRIX = {
    (0, 0): (2, 0),  # Cell 1-1
    (0, 1): (1, 0),  # 1-2
    (0, 2): (0, 0),  # 1-3

    (1, 0): (2, 1),  # 2-1
    (1, 1): (1, 1),  # 2-2
    (1, 2): (0, 1),  # 2-3

    (2, 0): (2, 2),  # 3-1
    (2, 1): (1, 2),  # 3-2
    (2, 2): (0, 2),  # 3-3
}

# Buzzer worker (non-overlapping)
_bot = None
_bot_lock = threading.Lock()

_beep_cv = threading.Condition()
_beep_queue = []
_beep_worker_started = False

def init_buzzer():
    global _bot, _beep_worker_started
    try:
        from Transbot_Lib import Transbot
        _bot = Transbot()
        rospy.loginfo("Buzzer ready: Transbot_Lib loaded.")
    except Exception as e:
        _bot = None
        rospy.logwarn("Buzzer disabled: cannot load Transbot_Lib (%s)" % str(e))
        return

    if not _beep_worker_started:
        t = threading.Thread(target=_beep_worker)
        t.daemon = True
        t.start()
        _beep_worker_started = True

def buzzer_off():
    if _bot is None:
        return
    with _bot_lock:
        try:
            if hasattr(_bot, "set_beep"):
                _bot.set_beep(0)
            if hasattr(_bot, "set_buzzer"):
                _bot.set_buzzer(0)
        except Exception:
            pass

def beep(times=1):
    if _bot is None:
        return
    times = int(max(1, min(times, 5)))
    with _beep_cv:
        if len(_beep_queue) < 20:
            _beep_queue.append(times)
            _beep_cv.notify()

def _beep_once():
    if _bot is None:
        return
    with _bot_lock:
        if hasattr(_bot, "set_beep"):
            _bot.set_beep(int(BEEP_ON_TIME_MS))
            return
        if hasattr(_bot, "set_buzzer"):
            _bot.set_buzzer(1)
            time.sleep(0.10)
            _bot.set_buzzer(0)

def _beep_worker():
    while not rospy.is_shutdown():
        with _beep_cv:
            while not _beep_queue and not rospy.is_shutdown():
                _beep_cv.wait(0.2)
            if rospy.is_shutdown():
                break
            times = _beep_queue.pop(0)

        try:
            for k in range(times):
                _beep_once()
                if k != times - 1:
                    time.sleep(BEEP_GAP_SEC)
        except Exception:
            pass

    buzzer_off()

def on_shutdown():
    buzzer_off()
    with _beep_cv:
        _beep_queue[:] = []

# Vision helpers
def order_points(pts):
    rect = np.zeros((4, 2), dtype="float32")
    s = pts.sum(axis=1)
    diff = np.diff(pts, axis=1)
    rect[0] = pts[np.argmin(s)]
    rect[2] = pts[np.argmax(s)]
    rect[1] = pts[np.argmin(diff)]
    rect[3] = pts[np.argmax(diff)]
    return rect

def find_board_quad(gray):
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blurred, 50, 150)

    kernel = np.ones((3, 3), np.uint8)
    edges = cv2.dilate(edges, kernel, iterations=1)

    cnts = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    contours = cnts[1] if len(cnts) == 3 else cnts[0]

    if not contours:
        return None

    h, w = gray.shape
    image_area = w * h
    contours = sorted(contours, key=cv2.contourArea, reverse=True)

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < 0.05 * image_area:
            continue

        peri = cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)

        if len(approx) == 4:
            pts = approx.reshape(4, 2)
            x, y, w_box, h_box = cv2.boundingRect(pts)
            aspect = float(w_box) / float(h_box) if h_box > 0 else 0
            if 0.7 < aspect < 1.3:
                return pts

    return None

def crop_cell_inner(cell_bgr):
    h, w = cell_bgr.shape[:2]

    if CELL_CROP_MODE == "px":
        mh = int(CELL_MARGIN_PX)
        mw = int(CELL_MARGIN_PX)
    else:
        mh = int(h * CELL_MARGIN_FRAC)
        mw = int(w * CELL_MARGIN_FRAC)

    mh = max(0, min(mh, h // 3))
    mw = max(0, min(mw, w // 3))
    return cell_bgr[mh:h - mh, mw:w - mw]

def extract_cells(warped_image):
    cell_images = []
    cell_h = warped_image.shape[0] // 3
    cell_w = warped_image.shape[1] // 3

    for i in range(3):
        row = []
        for j in range(3):
            cell = warped_image[i*cell_h:(i+1)*cell_h, j*cell_w:(j+1)*cell_w]
            row.append(crop_cell_inner(cell))
        cell_images.append(row)

    return cell_images

def avg_v_bgr(img_bgr):
    hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    return float(hsv[:, :, 2].mean())

def classify_by_v(v):
    if v <= V_X_MAX:
        return 'X'
    if v >= V_O_MIN:
        return 'O'
    if V_EMPTY_MIN <= v <= V_EMPTY_MAX:
        return '.'
    return '.'

def compute_matrix_from_cells(cells_3x3):
    mat = [['.' for _ in range(3)] for _ in range(3)]
    for i in range(3):
        for j in range(3):
            mr, mc = WIN_CELL_TO_MATRIX[(i, j)]
            v = avg_v_bgr(cells_3x3[i][j])
            mat[mr][mc] = classify_by_v(v)
    return mat

def matrix_key(mat):
    return tuple("".join(row) for row in mat)

def matrix_to_string(mat):
    # "XO./.X./..O" (rows joined by "/")
    return "/".join("".join(row) for row in mat)

def print_matrix(mat, header=None):
    if header:
        print("\n" + header)
    for r in range(3):
        print("%s %s %s" % (mat[r][0], mat[r][1], mat[r][2]))
    sys.stdout.flush()

# State
_current_matrix = None
_last_matrix_key = None
_last_change_time = 0.0
_last_board_seen_time = 0.0

_pub = None

# ROS callbacks
def image_callback(msg):
    global _current_matrix, _last_matrix_key, _last_change_time, _last_board_seen_time, _pub

    frame = bridge.imgmsg_to_cv2(msg, "bgr8")
    display = frame.copy()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    quad = find_board_quad(gray)

    if quad is not None:
        _last_board_seen_time = time.time()

        cv2.polylines(display, [quad], isClosed=True, color=(0, 255, 0), thickness=2)

        src_pts = order_points(quad)
        dst_pts = np.array([
            [0, 0],
            [WARP_SIZE - 1, 0],
            [WARP_SIZE - 1, WARP_SIZE - 1],
            [0, WARP_SIZE - 1]
        ], dtype="float32")

        M = cv2.getPerspectiveTransform(src_pts.astype("float32"), dst_pts)
        warped = cv2.warpPerspective(frame, M, (WARP_SIZE, WARP_SIZE))

        cells = extract_cells(warped)

        for i in range(3):
            for j in range(3):
                cv2.imshow("Cell {}-{}".format(i+1, j+1), cells[i][j])
        cv2.imshow("TTT Auto Detected Board (original)", display)
        cv2.imshow("TTT Warped Board (top-down)", warped)

        mat = compute_matrix_from_cells(cells)
        key = matrix_key(mat)
        _current_matrix = mat

        now = time.time()
        if _last_matrix_key is None:
            _last_matrix_key = key
            # publish first seen board too
            if _pub is not None:
                _pub.publish(String(data=matrix_to_string(_current_matrix)))
            print_matrix(_current_matrix, header="[INIT] Matrix:")
        else:
            if key != _last_matrix_key and (now - _last_change_time) >= CHANGE_DEBOUNCE_SEC:
                _last_change_time = now
                _last_matrix_key = key

                # Double buzz on change
                beep(times=2)
                print_matrix(_current_matrix, header="[CHANGED] Matrix:")

                # Publish on change
                if _pub is not None:
                    _pub.publish(String(data=matrix_to_string(_current_matrix)))

    else:
        cv2.imshow("TTT Auto Detected Board (original)", display)

    cv2.waitKey(1)

def periodic_timer_cb(event):
    global _current_matrix, _last_board_seen_time, _pub

    if _current_matrix is None:
        return

    now = time.time()
    if (now - _last_board_seen_time) > BOARD_SEEN_TIMEOUT_SEC:
        print("\n[PERIODIC] Board not detected recently.")
        sys.stdout.flush()
        return

    # Single buzz every PERIODIC_SEC
    beep(times=1)
    print_matrix(_current_matrix, header="[PERIODIC] Matrix:")

    # Optional periodic publish (handy for late subscribers)
    if _pub is not None:
        _pub.publish(String(data=matrix_to_string(_current_matrix)))

def main():
    global _pub

    rospy.init_node("ttt_board_state_pub")
    rospy.loginfo("ttt_board_state_pub started. Publishing /ttt/board_matrix as String rows joined by '/'.")

    rospy.on_shutdown(on_shutdown)
    init_buzzer()

    _pub = rospy.Publisher("/ttt/board_matrix", String, queue_size=10)

    rospy.Subscriber(CAMERA_TOPIC, Image, image_callback)
    rospy.Timer(rospy.Duration(PERIODIC_SEC), periodic_timer_cb)

    rospy.spin()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
