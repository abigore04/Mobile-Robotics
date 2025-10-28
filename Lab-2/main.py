#!/usr/bin/env python3

from ev3dev2.motor import MoveTank, SpeedPercent, OUTPUT_B, OUTPUT_C
from ev3dev2.sensor import INPUT_2, INPUT_3
from ev3dev2.sensor.lego import ColorSensor, GyroSensor
from time import sleep, time


mT =MoveTank(OUTPUT_B, OUTPUT_C)
cS =ColorSensor(INPUT_3)
cS.mode = 'COL-REFLECT'
gS =GyroSensor(INPUT_2)

LINE_MAX =24
CROSS_MIN,CROSS_MAX = 24,40
WHITE_MIN =70

TARG_LITE =9
BASE_M_SPD =20
kP =0.3
kD =3.0
MAX_T  =24
SLEW_RATE =6
DT = 0.025

lite_hist =[cS.reflected_light_intensity]*3
cross_count =0
x_streak =0
next_x_time = 0.0
last_e = 0.0
last_turning_val = 0.0


def reset_gyro():
    gS.mode= 'GYRO-RATE';sleep(0.15)
    gS.mode= 'GYRO-ANG';sleep(0.15)

def spin_360_deg():
    reset_gyro()
    mT.on(SpeedPercent(33),SpeedPercent(-33))
    while abs(gS.angle)< 360:
        sleep(0.005)
    mT.off(brake=True)

def drive_a_bit_frwrd():
    mT.on(SpeedPercent(19),SpeedPercent(19))
    sleep(0.6)
    mT.off(brake=True)

def g_turn(direction_sign, degs_to_oscill, sp_to_oscill):
    degs_to_oscill+=5
    start_ang =gS.angle
    left_m_spd =sp_to_oscill * direction_sign
    right_m_spd = -sp_to_oscill * direction_sign
    mT.on(SpeedPercent(left_m_spd), SpeedPercent(right_m_spd))
    while True:
        curr_ang =gS.angle
        if cS.reflected_light_intensity<= LINE_MAX:
            mT.off(brake=True);sleep(0.1)
            return True
        if abs(curr_ang -start_ang) >= degs_to_oscill:
            break
        sleep(0.005)
    mT.off(brake=True)
    return False

def hunt_for_line():
    mT.on(SpeedPercent(25),SpeedPercent(25))
    sleep(0.25)
    mT.off(brake=True)
    reset_gyro()
    angles =[30, 60, 90, 120, 150, 180]
    for angle in angles:
        if g_turn(1,angle,20): return True
        if g_turn(-1, 2*angle,20): return True
        if g_turn(1,angle,20): return True
    return False


try:
    while True:
        cur_l = cS.reflected_light_intensity
        lite_hist.pop(0); lite_hist.append(cur_l)
        avg_lite = sum(lite_hist)/len(lite_hist)
        now =time()

        if avg_lite>WHITE_MIN:
            mT.off(brake=True)
            if not hunt_for_line():
                break
            next_x_time = time() +0.3 
            last_e =0.0; last_turning_val =0.0
            continue

        if now >= next_x_time and CROSS_MIN <= avg_lite<= CROSS_MAX:
            x_streak+=1
        else:
            x_streak=0

        if x_streak>=2:
            x_streak=0
            cross_count+=1
            mT.off(brake=True)

            if cross_count==1:
                spin_360_deg()
                drive_a_bit_frwrd()
                next_x_time = time()+0.6
                last_e=0.0; last_turning_val=0.0
            elif cross_count==2:
                mT.off(brake=True)
                sleep(0.2)
                break

        if cross_count<2:
            err =TARG_LITE-avg_lite
            deriv =err-last_e
            turn_calculation = kP*err+kD*deriv

            if turn_calculation>MAX_T: turn_calculation =MAX_T
            if turn_calculation< -MAX_T: turn_calculation = -MAX_T

            min_s = last_turning_val- SLEW_RATE
            max_s = last_turning_val+ SLEW_RATE
            final_turning_value = max(min_s,min(turn_calculation, max_s))

            last_turning_val =final_turning_value
            last_e =err

            left = BASE_M_SPD +final_turning_value
            right = BASE_M_SPD -final_turning_value

            left = max(-100,min(left, 100))
            right = max(-100,min(right, 100))

            mT.on(SpeedPercent(left),SpeedPercent(right))
            sleep(DT)
        else:
            break

except KeyboardInterrupt:
    print("has interrupted mannually.")
finally:
    mT.off(brake=True)
