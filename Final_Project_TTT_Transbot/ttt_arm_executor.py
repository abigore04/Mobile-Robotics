#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import rospy
from std_srvs.srv import Trigger, TriggerResponse
from ttt_vision.srv import GotoCell, GotoCellResponse

from Transbot_Lib import Transbot

# Home
HOME = {"s7": 38,"s8": 230,"s9": 170}
HOME_PAN= 90

# Poses
POSES = {
    "playground": {
        (0, 0): (127,155,170,110),
        (0, 1): (126,157,170,100),
        (0, 2): (127,155,170,86),
        (1, 0): (124,172,170,113),
        (1, 1): (124,174,170,100),
        (1, 2): (124,172,170,83),
        (2, 0): (124,187,170,117),
        (2, 1): (124,189,170,100),
        (2, 2): (124,187,170,80),
    },
    "storage": {
        (0, 0): (130,144,170,69),
        (0, 1): (134,133,170,60),
        (0, 2): (143,110,170,54),
        (1, 0): (125,162,170,64),
        (1, 1): (128,150,170,55),
        (1, 2): (133,135,170,49),
        (2, 0): (124,175,170,58),
        (2, 1): (125,164,170,49),
        (2, 2): (128,150,170,42),
    }
}

# Safety Limits
S7_MIN,S7_MAX = 0,225
S8_MIN, S8_MAX = 30,270
S9_MIN, S9_MAX = 30,180
PAN_MIN, PAN_MAX = 0,180

# Servo and Pan Speeds
RT_S8_MS =2200
RT_S7_MS = 2200
PAN_STEP_DEG = 1
PAN_STEP_DELAY_S = 0.02
ARM_SETTLE_S = 0.15

PAN_SERVO_ID = 1  

def clamp(x, lo, hi):
    return max(lo, min(hi, int(x)))

def smooth_pan(bot, servo_id, current, target, step_deg=1, delay_s=0.02):
    current =int(current)
    target= int(target)
    step = max(1,int(abs(step_deg)))
    delay_s = max(0.0,float(delay_s))

    if current==target:
        bot.set_pwm_servo(servo_id, target)
        return target

    direction = 1 if target > current else -1
    a = current
    while a != target and not rospy.is_shutdown():
        nxt = a + direction*step
        if (direction > 0 and nxt > target) or (direction < 0 and nxt < target):
            nxt = target
        bot.set_pwm_servo(servo_id, nxt)
        a = nxt
        time.sleep(delay_s)
    return target

def arm_cmd(bot, s7, s8, s9, runtime_ms):
    s7c = clamp(s7,S7_MIN,S7_MAX)
    s8c = clamp(s8,S8_MIN,S8_MAX)
    s9c = clamp(s9,S9_MIN,S9_MAX)
    bot.set_uart_servo_angle_array(int(s7c), int(s8c), int(s9c), int(runtime_ms))
    time.sleep(int(runtime_ms) / 1000.0 + ARM_SETTLE_S)
    return s7c,s8c,s9c

class ArmExecutor:
    def __init__(self):
        self.bot = Transbot()
        time.sleep(0.1)

        self.bot.set_uart_servo_torque(1)

        self.state = {"s7": HOME["s7"], "s8": HOME["s8"], "s9": HOME["s9"], "pan": HOME_PAN}

        self.bot.set_pwm_servo(PAN_SERVO_ID, clamp(HOME_PAN, PAN_MIN, PAN_MAX))
        time.sleep(0.05)
        self.state["s7"],self.state["s8"],self.state["s9"] = arm_cmd(
            self.bot,HOME["s7"],HOME["s8"],HOME["s9"],max(RT_S7_MS, RT_S8_MS)
        )

        rospy.loginfo("ArmExecutor READY")

    def home_to_cell(self, target_pose):
        ts7,ts8,ts9, tpan = target_pose

        pan_target = clamp(tpan,PAN_MIN,PAN_MAX)
        self.state["pan"] = smooth_pan(self.bot,PAN_SERVO_ID, self.state["pan"],pan_target,PAN_STEP_DEG,PAN_STEP_DELAY_S)

        s7_hold = HOME["s7"]
        self.state["s7"],self.state["s8"],self.state["s9"] = arm_cmd(self.bot,s7_hold,ts8,ts9,RT_S8_MS)

        self.state["s7"],self.state["s8"], self.state["s9"] = arm_cmd(self.bot,ts7,ts8,ts9,RT_S7_MS)

    def cell_to_home(self):
        hs7,hs8,hs9 = HOME["s7"], HOME["s8"], HOME["s9"]

        s8_hold = self.state["s8"]
        self.state["s7"], self.state["s8"], self.state["s9"] = arm_cmd(self.bot,hs7,s8_hold,hs9,RT_S7_MS)

        self.state["s7"], self.state["s8"], self.state["s9"] = arm_cmd(self.bot,hs7,hs8,hs9,RT_S8_MS)

        pan_target = clamp(HOME_PAN, PAN_MIN, PAN_MAX)
        self.state["pan"] = smooth_pan(self.bot, PAN_SERVO_ID, self.state["pan"],pan_target,PAN_STEP_DEG,PAN_STEP_DELAY_S)

    def srv_go_home(self, req):
        try:
            self.cell_to_home()
            return TriggerResponse(success=True,message="OK: HOME")
        except Exception as e:
            return TriggerResponse(success=False,message="ERR: %s" % str(e))

    def _srv_goto(self, board, row, col):
        if (row, col) not in POSES[board]:
            return GotoCellResponse(ok=False,msg="Invalid %s cell (%d,%d)" % (board, row, col))
        try:
            self.cell_to_home()
            self.home_to_cell(POSES[board][(row, col)])
            return GotoCellResponse(ok=True,msg="OK: %s (%d,%d)" % (board, row, col))
        except Exception as e:
            return GotoCellResponse(ok=False,msg="ERR: %s" % str(e))

    def srv_goto_playground(self, req):
        return self._srv_goto("playground",req.row,req.col)

    def srv_goto_storage(self, req):
        return self._srv_goto("storage",req.row,req.col)

def main():
    rospy.init_node("ttt_arm_executor")

    exe =ArmExecutor()

    rospy.Service("/ttt/arm/go_home",Trigger, exe.srv_go_home)
    rospy.Service("/ttt/arm/goto_playground",GotoCell, exe.srv_goto_playground)
    rospy.Service("/ttt/arm/goto_storage",GotoCell, exe.srv_goto_storage)

    rospy.loginfo("Services up: /ttt/arm/go_home, /ttt/arm/goto_playground, /ttt/arm/goto_storage")
    rospy.spin()

if __name__ == "__main__":
    main()
