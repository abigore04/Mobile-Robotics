#!/usr/bin/env python3

from ev3dev2.motor import MoveTank, MediumMotor, SpeedPercent, OUTPUT_B, OUTPUT_C, OUTPUT_D
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3, INPUT_4
from ev3dev2.sensor.lego import UltrasonicSensor, GyroSensor, ColorSensor
from ev3dev2.sound import Sound
from time import sleep, time


mT =MoveTank(OUTPUT_B, OUTPUT_C)
mArm =MediumMotor(OUTPUT_D)
uS =UltrasonicSensor(INPUT_1)
gS =GyroSensor(INPUT_2)
cS =ColorSensor(INPUT_3) 
lS =ColorSensor(INPUT_4)
spk = Sound()

cS.mode= 'COL-REFLECT'
lS.mode = 'COL-REFLECT'

LINE_MAX= 30
WHITE_MIN =90
TARG_LITE= 60
BASE_M_SPD = 30
kP = 0.08
kD= 5
MAX_T = 30
SLEW_RATE= 0
DT = 0.0025
STOP_CM =5.0
SCAN_THRESHOLD = 25.0

lite_hist= [cS.reflected_light_intensity]*3
last_e =0.0
last_turning_val =0.0
box_counter = 0

def reset_gyro():
    gS.mode = 'GYRO-RATE';sleep(0.15)
    gS.mode = 'GYRO-ANG';sleep(0.15)

def smooth_stop(decay_time=0.005, step=0.0025):
    spd =30
    while spd >0:
        mT.on(SpeedPercent(spd), SpeedPercent(spd))
        spd -= 30*step/decay_time
        sleep(step)
    mT.off(brake=False)
    sleep(0.05)

def g_turn(direction_sign, degs_to_oscill, sp_to_oscill):
    start_ang =gS.angle
    left_spd = sp_to_oscill*direction_sign
    right_spd = -sp_to_oscill*direction_sign
    mT.on(SpeedPercent(left_spd), SpeedPercent(right_spd))
    while True:
        cur_ang = gS.angle
        if cS.reflected_light_intensity<= LINE_MAX:
            mT.off(brake=True)
            return True
        if abs(cur_ang-start_ang) >=degs_to_oscill:
            break
        sleep(0.005)
    mT.off(brake=True)
    return False

def hunt_for_line():
    reset_gyro()
    angles= [70, 80, 110]
    for angle in angles:
        if g_turn(-1,angle,35): return True
        if g_turn(1,2*angle,35): return True
        if g_turn(-1,angle,35): return True
    return False

def obstacle_alarm():
    global box_counter
    smooth_stop()

    start = time()
    while time()-start < 3:
        spk.tone(1000, 300)
        sleep(0.1)

    mArm.stop_action ='brake'
    mArm.on_for_seconds(SpeedPercent(90), 0.3)
    mArm.on_for_seconds(SpeedPercent(-90), 0.3)
    box_counter += 1

def spin_180():
    reset_gyro()
    mT.on(SpeedPercent(35), SpeedPercent(-35))
    while abs(gS.angle)< 180:
        sleep(0.005)
    mT.off(brake=True)
    sleep(0.2)

def end_alarm():
    start =time()
    while time()-start < 2:
        spk.tone(400, 100)
        sleep(0.2)

sleep(0.3)
start_time= time()
while time()-start_time< 0.2:
    mT.on(SpeedPercent(30), SpeedPercent(30))
    sleep(0.01)
smooth_stop()

try:
    while True:
        if uS.distance_centimeters <=STOP_CM:
            obstacle_alarm()
            last_e =0.0
            last_turning_val = 0.0
            continue

        if box_counter>=1 and uS.distance_centimeters<=SCAN_THRESHOLD:
            spin_180()
            last_e =0.0
            last_turning_val =0.0
            continue

        cur_l = cS.reflected_light_intensity
        cur_side_l = lS.reflected_light_intensity
        lite_hist.pop(0); lite_hist.append(cur_l)
        avg_lite = sum(lite_hist)/len(lite_hist)

        if cur_side_l<=12:
            smooth_stop()
            reset_gyro()
            mT.on(SpeedPercent(-35), SpeedPercent(35))
            while abs(gS.angle) < 15:
                sleep(0.005)
            smooth_stop()
            continue

        if cS.reflected_light_intensity> WHITE_MIN and lS.reflected_light_intensity> WHITE_MIN:
            smooth_stop()
            if not hunt_for_line():
                end_alarm()
                break
            last_e =0.0
            last_turning_val= 0.0
            continue

        err = TARG_LITE-avg_lite
        deriv = err-last_e
        turn_calculation = kP*err + kD*deriv
        turn_calculation = max(-MAX_T, min(MAX_T, turn_calculation))

        min_s = last_turning_val -SLEW_RATE
        max_s = last_turning_val +SLEW_RATE
        final_turning_val = max(min_s, min(turn_calculation, max_s))

        last_turning_val =final_turning_val
        last_e =err

        left_spd = BASE_M_SPD+final_turning_val
        right_spd = BASE_M_SPD-final_turning_val
        left_spd = max(-100,min(left_spd, 100))
        right_spd = max(-100,min(right_spd, 100))

        mT.on(SpeedPercent(left_spd), SpeedPercent(right_spd))
        sleep(DT)

except KeyboardInterrupt:
    pass
finally:
    smooth_stop()
