#!/usr/bin/env python3

from ev3dev2.motor import MoveTank, SpeedPercent, OUTPUT_B, OUTPUT_C
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3
from ev3dev2.sensor.lego import ColorSensor, GyroSensor
from time import sleep, time

import math, csv

mT =MoveTank(OUTPUT_B, OUTPUT_C)
cS =ColorSensor(INPUT_3)
cS.mode = 'COL-REFLECT'
gS =GyroSensor(INPUT_2)
gS2 = GyroSensor(INPUT_1)

LINE_MAX =24
CROSS_MIN, CROSS_MAX = 24, 40
WHITE_MIN =70

TARG_LITE =9
BASE_M_SPD =20
kP =0.3
kD =3.0
MAX_T =24
SLEW_RATE =6
DT = 0.025


lite_hist =[cS.reflected_light_intensity]*3
cross_count =0
x_streak =0
next_x_time = 0.0
last_e =0.0 
last_turning_val = 0.0

WHEEL_DIAMETER_CM =5.6
DIST_PER_DEG_CM =(math.pi* WHEEL_DIAMETER_CM)/ 360.0  

x_axis_cm, y_axis_cm =0.0,0.0
last_deg_left =0
last_deg_right= 0
DO_LOGGING =True

log_file= open("path_log.csv", "w", newline="")
csv_writer= csv.writer(log_file)
csv_writer.writerow(["time", "x_axis_cm", "y_axis_cm", "heading_deg", "cross"])

def reset_gyro():
    gS.mode= 'GYRO-RATE';sleep(0.15)
    gS.mode= 'GYRO-ANG';sleep(0.15)

def reset_gyro2():
    gS2.mode= 'GYRO-RATE';sleep(0.15)
    gS2.mode= 'GYRO-ANG';sleep(0.15)

def log_pose():
    global x_axis_cm,y_axis_cm,last_deg_left,last_deg_right
    if not DO_LOGGING:
        return

    deg_left= mT.left_motor.position
    deg_right = mT.right_motor.position

    ddeg_left = deg_left-last_deg_left
    ddeg_right = deg_right-last_deg_right
    last_deg_left, last_deg_right = deg_left, deg_right

    ds_cm = 0.5 * (ddeg_left + ddeg_right) * DIST_PER_DEG_CM

    theta = math.radians(-gS2.angle)
    x_axis_cm += ds_cm*math.cos(theta)
    y_axis_cm += ds_cm*math.sin(theta)

    csv_writer.writerow([
        round(time()-START_T, 3),
        round(x_axis_cm,2),
        round(y_axis_cm,2),
        int(gS2.angle), 
        int(cross_count)
    ])
    
    log_file.flush()

def spin_360_deg():
    global DO_LOGGING
    DO_LOGGING= True
    reset_gyro()
    mT.on(SpeedPercent(33),SpeedPercent(-33))
    while abs(gS.angle)< 360:
        sleep(0.005)
        log_pose()
    mT.off(brake=True)

def drive_a_bit_frwrd():
    global DO_LOGGING
    DO_LOGGING = True
    mT.on(SpeedPercent(19),SpeedPercent(19))
    time0 =time()
    while time() - time0 < 0.6:
        sleep(0.02)
        log_pose()
    mT.off(brake=True)

def g_turn(direction_sign, degs_to_oscill, sp_to_oscill):
    degs_to_oscill+=5
    start_ang =gS.angle
    left_spd =sp_to_oscill * direction_sign
    right_spd = -sp_to_oscill * direction_sign
    mT.on(SpeedPercent(left_spd), SpeedPercent(right_spd))
    while True:
        curr_ang = gS.angle
        if cS.reflected_light_intensity<= LINE_MAX:
            mT.off(brake=True);sleep(0.1)
            return True
        if abs(curr_ang -start_ang) >= degs_to_oscill:
            break
        sleep(0.005)
    mT.off(brake=True)
    return False

def hunt_for_line():
    global DO_LOGGING
    DO_LOGGING =False

    mT.on(SpeedPercent(25),SpeedPercent(25))
    sleep(0.25)
    mT.off(brake=True)
    reset_gyro()
    angles = [30, 60, 90, 120, 150, 180]
    for angle in angles:
        if g_turn(1,angle,20): DO_LOGGING = True; return True
        if g_turn(-1,2*angle,20): DO_LOGGING = True; return True
        if g_turn(1,angle,20): DO_LOGGING = True; return True

    DO_LOGGING =True
    return False

START_T =time()
reset_gyro2()
last_deg_left = mT.left_motor.position
last_deg_right = mT.right_motor.position

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
            next_x_time = time() + 0.3
            last_e = 0.0; last_turning_val = 0.0
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
                last_e = 0.0; last_turning_val = 0.0

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
            last_e = err

            left =  BASE_M_SPD +final_turning_value
            right = BASE_M_SPD -final_turning_value

            left  = max(-100,min(left,100))
            right = max(-100,min(right, 100))

            mT.on(SpeedPercent(left), SpeedPercent(right))
            sleep(DT)
            log_pose()
        else:
            break

except KeyboardInterrupt:
    print("has interrupted mannually.")
finally:
    mT.off(brake=True)
    try:
        log_file.flush()
        log_file.close()
    except Exception:
        pass
