#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Base Calibration Script

import time
from Transbot_Lib import Transbot

POSE_A ={"s7": 38,"s8": 230,"s9": None} #None means keep current S9
POSE_B ={"s7": 123,"s8": 180,"s9": 107}

PAN_TARGET = 100

S7_MIN,S7_MAX = 0,225
S8_MIN,S8_MAX = 30,270
S9_MIN,S9_MAX = 30,180
PAN_MIN,PAN_MAX = 0,180

ARM_RUNTIME_MS =5000
PAN_STEP_DEG = 1
PAN_STEP_DELAY_S= 0.03

def clamp(x, lo, hi):
    return max( lo, min( hi, int(x) ) )

def wait_enter(msg):
    input(f"\n{msg}\nPress ENTER to execute...")

def smooth_pan(bot, servo_id, current, target, step_deg=1, delay_s=0.02):
    current = int(current)
    target = int(target)
    step = max(1, int(abs(step_deg)))
    delay_s = max(0.0, float(delay_s))

    if current == target:
        bot.set_pwm_servo(servo_id, target)
        return target

    direction =1 if target > current else -1
    a=current
    while a!=target:
        nxt = a+direction*step
        if (direction > 0 and nxt > target) or (direction < 0 and nxt < target):
            nxt = target
        bot.set_pwm_servo(servo_id, nxt)
        a = nxt
        time.sleep(delay_s)
    return target


def read_arm_angles(bot):
    ang = bot.get_uart_servo_angle_array()
    if isinstance(ang, (list, tuple)) and len(ang) == 3:
        return int(ang[0]), int(ang[1]), int(ang[2])
    return None


def move_arm(bot, s7, s8, s9, runtime_ms):
    cur = read_arm_angles(bot)
    if cur is None:
        cur = (90,160,110)  

    cur7, cur8, cur9 = cur
    if s9 is None:
        s9 = cur9

    s7c = clamp(s7,S7_MIN,S7_MAX)
    s8c = clamp(s8,S8_MIN,S8_MAX)
    s9c = clamp(s9,S9_MIN,S9_MAX)

    print(f"ARM readback before move: S7={cur7}, S8={cur8}, S9={cur9}")
    print(f"ARM command (clamped):    S7={s7c}, S8={s8c}, S9={s9c}, RT={runtime_ms}ms")

    bot.set_uart_servo_angle_array(s7c, s8c, s9c, int(runtime_ms))
    return (s7c, s8c, s9c)


def main():
    bot = Transbot()
    time.sleep(0.1)

    bot.set_uart_servo_torque(1)

    PAN_SERVO_ID = 1

    pan_current = 90
    bot.set_pwm_servo(PAN_SERVO_ID, pan_current)
    time.sleep(0.05)

    print("\n@@@ Calibration sequence: A -> PAN(100) -> B -> A -> END @@@")
    print(f"PAN_SERVO_ID = {PAN_SERVO_ID}")
    print(f"ARM_RUNTIME_MS = {ARM_RUNTIME_MS}")
    print(f"PAN smoothing: step={PAN_STEP_DEG} deg, delay={PAN_STEP_DELAY_S} s\n")

    cur = read_arm_angles(bot)
    print(f"Initial ARM angles: {cur if cur else 'read failed'}")

    # 1, arm -> POSE_A
    wait_enter(f"Step 1: ARM -> POSE_A (S7={POSE_A['s7']}, S8={POSE_A['s8']}, S9={'keep' if POSE_A['s9'] is None else POSE_A['s9']})")
    move_arm(bot, POSE_A["s7"],POSE_A["s8"],POSE_A["s9"], ARM_RUNTIME_MS)

    # 2. PAN -> 100
    wait_enter(f"Step 2: PAN -> {PAN_TARGET} (smoothed)")
    pan_target = clamp(PAN_TARGET, PAN_MIN, PAN_MAX)
    pan_current = smooth_pan(bot, PAN_SERVO_ID, pan_current, pan_target, PAN_STEP_DEG, PAN_STEP_DELAY_S)

    # 3. arm -> POSE_B
    wait_enter(f"Step 3: ARM -> POSE_B (S7={POSE_B['s7']}, S8={POSE_B['s8']}, S9={POSE_B['s9']})")
    move_arm(bot, POSE_B["s7"], POSE_B["s8"], POSE_B["s9"], ARM_RUNTIME_MS)

    # 4. arm -> POSE_A
    wait_enter("Step 4: ARM -> POSE_A (finish)")
    move_arm(bot, POSE_A["s7"], POSE_A["s8"], POSE_A["s9"], ARM_RUNTIME_MS)

    print("\nSequence complete. Terminating.")


if __name__ == "__main__":
    main()
