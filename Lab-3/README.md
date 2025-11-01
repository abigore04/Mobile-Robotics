# Line Following and Path Visualization using LEGO EV3 Mindstorms

**Course:** Mobile Robotics  
**Student:** Farid Ibadov  
**Instructor:** Alexzander Leonard Farral
**University:** ADA University  
**Date:** October 2025  
**Platform:** EV3Dev (Debian-based OS for LEGO EV3)  
**Language:** Python (EV3DEV2 library)

## Outline

- [Main Program](#main-program)
- [Program for Path Visualization](#program-for-path-visualization)
- [Objectives and Overview](#objectives-and-overview)
- [Theoretical Background](#theoretical-background)
- [Main Code Explanation](#main-code-explanation)
  - [1. Library Imports](#1-library-imports)
  - [2. Hardware Setup](#2-hardware-setup)
  - [3. Constants and Tuning](#3-constants-and-tuning)
  - [4. CSV Logging Initialization](#4-csv-logging-initialization)
  - [5. Helper Functions](#5-helper-functions)
    - [5.1 Resetting the Second Gyro](#51-resetting-the-second-gyro)
    - [5.2 Pose Logging](#52-pose-logging)
    - [5.3 360-Spin with Logging](#53-360-spin-with-logging)
    - [5.4 Moving a bit Forward with Logging](#54-moving-a-bit-forward-with-logging)
    - [5.5 Searching for Line when Got lost without Logging](#55-searching-for-line-when-got-lost-without-logging)
  - [6. Initializations before Main Loop](#6-initializations-before-main-loop)
  - [7. Main Loop](#7-main-loop)
  - [8. Safe Termination](#8-safe-termination)
- [Visualization Code Explanation](#visualization-code-explanation)
  - [1. Library Import](#1-library-import)
  - [2. Loading CSV](#2-loading-csv)
  - [3. Creating a Plot with a General Trajectory Visualization](#3-creating-a-plot-with-a-general-trajectory-visualization)
  - [4. Showing Start Point](#4-showing-start-point)
  - [5. Showing First Cross Point](#5-showing-first-cross-point)
  - [6. Showing End Point](#6-showing-end-point)
- [Summary](#summary)
- [YouTube Link](#youtube-link)
- [References](#references)

---

## Main Program

```python title:"wheel-based Lab-3 Code"
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
```

---

## Program for Path Visualization

```python title:"path visualization"
import pandas as pd
import matplotlib.pyplot as plt

path_of_file= "path_log.csv"
df = pd.read_csv(path_of_file)
print(df.head())

plt.figure(figsize=(300, 100))
plt.plot(df["x_axis_cm"], df["y_axis_cm"], "-o", linewidth=1, markersize=1, label="ev3's trajectory")

plt.scatter(df["x_axis_cm"].iloc[0], df["y_axis_cm"].iloc[0], color="green", s=60, label="ev3's woken up")

if "cross" in df.columns:
    cross1 = df[df["cross"] == 1]
    if not cross1.empty:
        first_cross1_reading = cross1.iloc[0]
        plt.scatter(first_cross1_reading["x_axis_cm"], first_cross1_reading["y_axis_cm"], color="black", s=100, label="360")

plt.scatter(df["x_axis_cm"].iloc[-1], df["y_axis_cm"].iloc[-1], color="red", s=80, label="ev3's went to sleep")

plt.title("Visualisation of path of ev3")
plt.xlabel("X in cm")
plt.ylabel("Y in cm")
plt.legend()
plt.axis("equal")
plt.grid(True)
plt.show()
```

---

## Objectives and Overview

In the following lab the objective is built on the logic of the previous lab, but with an additional functionality. Basically, the robot is expected to do exactly the same movements as he did before, on the same track, under the same conditions; however, this time, by collecting necessary data along his movement, it needs to implement the **odometry** and **data-logging** principles to recreate its path on 2D plot using mapping techniques.

So, in the following implementation:
	- **Odometry** was used to capture just displacement of the robot to understand how far the robot goes according to its motor sensors.
	- Understanding the **direction**, which robot was facing, was responsibility of the *second gyro*. 

## Theoretical Background

Before jumping straight into the code explanation, it would be useful to refer to the theoretical background and understand the core principles that were utilized in this lab.

The main login of the mapping of the path process relies on the **odometry**. In a nutshell, odometry is a use of motion sensors to determine the robot's change in position, relative to some known position. Using odometry we can compute:
	1. linear displacement.
	2. orientation change.

In this lab, I used the functionality of motor sensors to calculate linear displacement only, and left orientation tracking for the gyro. Although, it is possible to do that with motor sensors only, implementing pure odometry using wheels when our environment is spacious and track is complex may lead to the final result that doesn't correspond to the reality, due to sensor reading drifts, tire slipping, and so on. Pure odometry provides only short-term accuracy. Therefore, I thought that it would be better to introduce the second gyro and dedicate it specifically for one task - dynamic real-time robot's direction detection.

## Main Code Explanation

*Note: As I have mentioned before, the code in this lab is almost the same that were used in the previous lab, so I will avoid repeating myself and will focus on an **additional functionality.***

### 1. Library Imports

```python
#!/usr/bin/env python3

from ev3dev2.motor import MoveTank, SpeedPercent, OUTPUT_B, OUTPUT_C
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3
from ev3dev2.sensor.lego import ColorSensor, GyroSensor
from time import sleep, time

import math, csv
```

- In imports, we additionally introduce port `INPUT_1` where we connect our second gyro - `gS2`. Plus, we import `math` and `csv`, where the first is responsible for trigonometric operations that will be used for odometry calculations (like calculating sine, cosine, and radians) and the second - for logging the movement into csv file `path_log.csv`.

### 2. Hardware Setup

```python
mT =MoveTank(OUTPUT_B, OUTPUT_C)
cS =ColorSensor(INPUT_3)
cS.mode = 'COL-REFLECT'
gS =GyroSensor(INPUT_2)
gS2 = GyroSensor(INPUT_1)
```

- As mentioned before, here we additionally introduce the second gyro for global orientation tracking and specify that it is connected to the port 1 `gS2=GyroSensor(INPUT_1)`.

### 3. Constants and Tuning

```python
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
DIST_PER_DEG_CM =(math.pi * WHEEL_DIAMETER_CM)/ 360.0

x_axis_cm, y_axis_cm =0.0,0.0
last_deg_left =0
last_deg_right= 0
DO_LOGGING =True
```

In this part of our code we have 2 additional lines responsible for converting the motor rotations into linear distance measured in centimeters. By `WHEEL_DIAMETER_CM` we specify the diameter of standard EV3 wheel, which is 5.6 cm, and by `DIST_PER_DEG_CM =(math.pi* WHEEL_DIAMETER_CM)/ 360.0` we calculate the circumference of wheel divided by 360, which gives us the distance the robot passes per one-degree rotation of the wheel. 

Lastly, we have some variables and a flag used in data logging;
- `x_axis_cm` and `y_axis_cm` - are responsible for storing the robot's current position in relation of x and y axis. Initially set to **zero** to indicate the robot's starting point.
- `last_deg_left` and `last_deg_right` - those keep previous motor values and are used in calculation of the incrementation of movement at each step.
- `DO_LOGGING` - is a control flag which is to temporarily stop data logging when it is not needed (like when the robot gets lost and starts to oscillate). 

### 4. CSV Logging Initialization 

```python
log_file= open("path_log.csv", "w", newline="")
csv_writer= csv.writer(log_file)
csv_writer.writerow(["time", "x_axis_cm", "y_axis_cm", "heading_deg", "cross"])
```

- `log_file= open("path_log.csv", "w", newline="")`
		- open/create in a writing mode a file named `path_log.csv`
		- `newline=""` to avoid blank lines between actual data lines in csv file.

- `csv_writer`
		- a csv writer object

- `csv_writer.writerow(["time", "x_axis_cm", "y_axis_cm", "heading_deg", "cross"])`
		- write the first row as a header to specify which column corresponds to which parameter.
		- columns' names are self-descriptive.
		- as a result we will get something like this:
```python
time,x_axis_cm,y_axis_cm,heading_deg,cross
0.388,0.05,0.0,0,0
0.456,0.64,0.0,0,0
0.519,1.25,0.01,-1,0
0.584,2.03,0.04,-2,0
0.65,2.71,0.06,-2,0
0.722,3.42,0.09,-2,0
0.798,4.2,0.11,-2,0
0.862,4.88,0.14,-2,0
```

### 5. Helper Functions

#### 5.1 Resetting the Second Gyro

```python
def reset_gyro2():
    gS2.mode= 'GYRO-RATE';sleep(0.15)
    gS2.mode= 'GYRO-ANG';sleep(0.15)
```

- clear.

#### 5.2 Pose Logging

- This is the essential function for data logging.

```python
def log_pose():
    global x_axis_cm,y_axis_cm,last_deg_left,last_deg_right
    if not DO_LOGGING:
        return       # to exit the func when logging is not needed.

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
```

- `deg_left` and `deg_right` - current motors' positions (rotation angles).
- `ddeg_left` and `ddeg_right` - difference in angles since last call to get incremental rotation value for each separate wheel.
- `last_deg_left` and `last_deg_right` - update last value.

- Based on the differential-drive odometry formula (Eq. 5.5 in [1]):
- $$\Delta S = \frac{\Delta S_r - \Delta S_l}{2}$$
	- $\Delta S$ = linear displacement of the center of robot.
	- $\Delta S_r - \Delta S_l$ = traveled distances for the right and left wheel respectively;
- we can write it as `ds_cm = 0.5 * (ddeg_left + ddeg_right) * DIST_PER_DEG_CM` to get the linear displacement of the center of robot.

- So, to calculate the change in the robot's movement on a 2D plot in relation to the x and y axis we have the following formulas (Eq. 5.2, 5.3 in [1]):
  - $$\Delta x = \Delta S\cdot \cos(\theta + \frac{\Delta \theta}{2})$$
  - $$\Delta y = \Delta S\cdot \sin(\theta + \frac{\Delta \theta}{2})$$
- However, since I have decided to use second gyro for orientation tracking, we can rewrite those formulas like this:
	- since  $$\theta_{gS2} = \theta + \frac{\Delta \theta}{2}$$
 - we just write:
     - $$\Delta x = \Delta S\cdot \cos(\theta_{gS2})$$
     - $$\Delta y = \Delta S\cdot \sin(\theta_{gS2})$$
 - therefore we have:
```python
theta = math.radians(-gS2.angle)
x_axis_cm += ds_cm*math.cos(theta)
y_axis_cm += ds_cm*math.sin(theta)
```

```python
csv_writer.writerow([
	round(time()-START_T, 3),
	round(x_axis_cm,2),
	round(y_axis_cm,2),
	int(gS2.angle), 
	int(cross_count)
])
```

- By `csv_writer.writerow([...])` command, every time we call it, a new row is added to csv file with the following element to each column:
	- `time()-START_T` - time passed since the start of the program, rounded to 3 decimal places
	- `x_axis_cm` - current x coordinate, rounded to 2 decimal places.
	- `y_axis_cm` - current y coordinate, rounded to 2 decimal places.
	- `gS2.angle` - angle provided by the second gyro - where robot heads, in integer value.
	- `cross_count` - when 0 - no cross detected, 1 - first cross (360-spin), 2 - the second cross (stop), in integer value.

- By `log_file.flush()` we force writing from a memory buffer to the actual file, avoiding data loss in cases such as interruption or crash.

#### 5.3 360-Spin with Logging

```python
def spin_360_deg():
    global DO_LOGGING
    DO_LOGGING= True
    reset_gyro()
    mT.on(SpeedPercent(33),SpeedPercent(-33))
    while abs(gS.angle)< 360:
        sleep(0.005)
        log_pose()
    mT.off(brake=True)
```

- The same function for 360-spin, but with additional functionality of logging the motion in real time.

- When we write `global DO_LOGGING` in our function we are using the same `DO_LOGGING` variable from the global scope of our program; doing so, the function can actually modify its value and affect logging behavior globally, not just locally.

- By `DO_LOGGING = True` we can imagine some kind of switch that says where the data should and should not be recorded. Here we have `=True`, meaning we actually want the data to be logged during 360-spin maneuver.

```python
while abs(gS.angle)< 360:
        sleep(0.005)
        log_pose()
```

- In our 360-spin loop, the logging interval is set to 5 milliseconds, meaning we will log the data using `log_pose()` function every 0.005 seconds.

- *Note: Later I have decided just to mark the 360-spin point with a single marker, since I did not like the visual result that was provided by "drawing a circle".* 

#### 5.4 Moving a bit Forward with Logging

```python
def drive_a_bit_frwrd():
    global DO_LOGGING
    DO_LOGGING = True
    mT.on(SpeedPercent(19),SpeedPercent(19))
    time0 =time()
    while time() - time0 < 0.6:
        sleep(0.02)
        log_pose()
    mT.off(brake=True)
```

- here the logging-function-call logic is the same as in `spin_360_deg()` function. 

- *Reminder*: this function was used to go off the cross tape once 360-spin is performed to avoid second detection of the same tape. 

#### 5.5 Searching for Line when Got lost without Logging

- Since `g_turn()` function is called from `hunt_for_line()` function, there is no logging-stuff there, instead, in `hunt_for_line()` function:

```python
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
```

- we specify `DO_LOGGING =False`, meaning, it is not desirable to log the oscillations, since they're not related to the path-plotting logic.
- Only when `g_turn()` function returns `True` we specify `DO_LOGGING = True`, meaning, we returned to the main line successfully and we want to resume our data logging. 
- If we fail in line search, the logging turns on anyway - to record the place where the robot has stopped before turning off the program.

### 6. Initializations before Main Loop

- Before we start our program by running the main loop, it is important to make sure that we initialize and reset some parameters.
```python
START_T =time()
reset_gyro2()
last_deg_left = mT.left_motor.position
last_deg_right = mT.right_motor.position
```

- by `START_T =time()` we capture the time when our program has started.
- by `reset_gyro2()` we reset our second gyro just once - when it faces the East (positive x-axis) to be informed where our robot heads at any time during its movement.
- finally, we read and store the initial wheel angles to be able to calculate the incremental displacement of the robot correctly later.


### 7. Main Loop

```python
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
```

- Our main loop has almost stayed intact, except 1 new line in the very bottom - `log_pose()`. It is placed at the very bottom of our main loop in order to record the position of the robot, its heading direction, and the number cross-lines he crossed after each complete update, which takes 25 milliseconds (DT = 0.025), to make sure that every displacement data is logged by the end of each loop iteration.

### 8. Safe Termination

- The final part in our program is the so-called "Safe Termination" block which ensures that our robot stops safely and the data it have logged is saved properly in case if program is interrupted manually or unexpectedly crashes.

```python
except KeyboardInterrupt:
    print("has interrupted mannually.")
finally:
    mT.off(brake=True)
    try:
        log_file.flush()
        log_file.close()
    except Exception:
        pass
```

- When we stop the program manually, that event is caught using `except KeyboardInterrupt:` and the message "has interrupted manually." prints to the console.

- `finally:` block always runs. We then turn off the motors and safely write all data to the file using `log_file.flush()` and close the csv file `log_file.close()`. 

## Visualization Code Explanation

- As soon as we collect the data, it is important to figure out how to visualize it. I have done it in a following way:
	1. Wrote a program for visualization.
	2. Uploaded the .csv file from the Brick's memory into my code editor.
	3. And run it on my pc. 

- Now it is important to understand how the visualization code works

```python
import pandas as pd
import matplotlib.pyplot as plt

path_of_file= "path_log.csv"
df = pd.read_csv(path_of_file)
print(df.head())

plt.figure(figsize=(300, 100))
plt.plot(df["x_axis_cm"], df["y_axis_cm"], "-o", linewidth=1, markersize=1, label="ev3's trajectory")

plt.scatter(df["x_axis_cm"].iloc[0], df["y_axis_cm"].iloc[0], color="green", s=60, label="ev3's woken up")

if "cross" in df.columns:
    cross1 = df[df["cross"] == 1]
    if not cross1.empty:
        first_cross1_reading = cross1.iloc[0]
        plt.scatter(first_cross1_reading["x_axis_cm"], first_cross1_reading["y_axis_cm"], color="black", s=100, label="360")

plt.scatter(df["x_axis_cm"].iloc[-1], df["y_axis_cm"].iloc[-1], color="red", s=80, label="ev3's went to sleep")

plt.title("Visualisation of path of ev3")
plt.xlabel("X in cm")
plt.ylabel("Y in cm")
plt.legend()
plt.axis("equal")
plt.grid(True)
plt.show()
```

### 1. Library Import

We import `pandas` and `matplotlib.pyplot`, where the first is used to read the csv file, and the second - to plot the actual path out of data that the robot has logged during its motion.

### 2. Loading CSV

```python
path_of_file= "path_log.csv"
df = pd.read_csv(path_of_file)
print(df.head())
```

- By `path_of_file= "path_log.csv"` we access the `path_log.csv` to be able to read the data later.
- As we know, we have 5 `csv_writer.writerow(["time", "x_axis_cm", "y_axis_cm", "heading_deg", "cross"])` columns for 5 types of data logged and we will manipulate those data to construct descriptive 2d-plot.
- By `pd.read_csv(path_of_file)` we read csv file and then return a dataframe `df` structure
- This `print(df.head())` is optional and is used only to print first several rows of data in terminal before plotting, just for us to make sure that the data is read correctly.

### 3. Creating a Plot with a General Trajectory Visualization

```python
plt.figure(figsize=(300, 100))
plt.plot(df["x_axis_cm"], df["y_axis_cm"], "-o", linewidth=1, markersize=1, label="ev3's trajectory")
```

- Using `plt.figure(figsize=(300, 100))` a plot with a width of 3 meters and a height of 1 meter is created, to imitate the actual dimensions of the table.
- With `plt.plot(...)` we draw a continuous line representing the robot's general movement (except when "hunting" for a line) by reading x and y coordinates from csv, marking all logged points using `-o`, drawing thin line and little dots using `linewidth` and `markersize` and setting them both to 1, and with `label="ev3's trajectory"`, giving the label, so someone else understands what is that line for.

### 4. Showing Start Point

```python
plt.scatter(df["x_axis_cm"].iloc[0], df["y_axis_cm"].iloc[0], color="green", s=60, label="ev3's woken up")
```

- Now we use `plt.scatter` instead of `plt.plot` to draw single point - marker. And we explicitly specify to do this only referring to the first data-log process, meaning, the first row in our csv. Color green, size is 60, and descriptive label

### 5. Showing First Cross Point

```python
if "cross" in df.columns:
    cross1 = df[df["cross"] == 1]
    if not cross1.empty:
        first_cross1_reading = cross1.iloc[0]
        plt.scatter(first_cross1_reading["x_axis_cm"], first_cross1_reading["y_axis_cm"], color="black", s=100, label="360")
```

- Here the logic is almost the same as in start point visualization, except we specify to visualize the first log when cross parameter is 1, to ensure the precise point where the robot performed 360-spin. And indeed, if we look at visualized movement obtained after running the main code and then running the visualizer code:
<img width="1913" height="1017" alt="Pasted image 20251101113223" src="https://github.com/user-attachments/assets/f967839e-b8c7-4a92-81ea-614ba2e82bb9" />


- we can observe that the robot has performed the 360-spin in ~120 cm away from the starting point. And if we take a ruler and physically measure the distance between start and 360-spin point, the actual distance is very close to the measured one.
<img width="2560" height="1440" alt="Pasted image 20251101113505" src="https://github.com/user-attachments/assets/e3a2b0c6-5b09-4a1f-ae13-1d9e31d339d6" />


- *Note: I've made sure that the robot start from exact position every time:*
<img width="2543" height="1689" alt="Pasted image 20251101113816" src="https://github.com/user-attachments/assets/5771f568-9892-4944-b260-995ea875bf46" />


### 6. Showing End Point

```python
plt.scatter(df["x_axis_cm"].iloc[-1], df["y_axis_cm"].iloc[-1], color="red", s=80, label="ev3's went to sleep")
```

- Again, similar plotting behavior, just using `iloc[-1]` we make sure that only the final logged row is depicted.

### 7. Customization and Text Display

```python
plt.title("Visualisation of path of ev3")
plt.xlabel("X in cm")
plt.ylabel("Y in cm")
plt.legend()
plt.axis("equal")
plt.grid(True)
plt.show()
```

- Adding general title with `plt.title`
- Showing which axis is which using `plt.xlabel` and `plt.ylabel`
- `plt.legend` displays all labels from plotting commands above
- `plt.axis` to make axis proportional
- `plt.grid` adding some grid for a pleasant view
- `plt.show` to visualize the final result


## Summary

Overall, the objective of this lab is similar to the objective of the previous one's. The EV3 robot had to move along green line, to perform 360-spin when the first grey cross-line is detected and to stop on the second one. However, here one of the important concepts of robotics - Localization and Mapping were introduced and practically understood. Using simple logging techniques and trivial to-do/not-to-do pattern, robot achieved the goal of visualizing its path with an acceptable precision.  

## YouTube Link

https://youtu.be/zhpJnz-qXio

## References

[1] R. Siegwart and I. R. Nourbakhsh, _Introduction to Autonomous Mobile Robots_, 2nd ed. Cambridge, MA: MIT Press, 2011, p. 186-7.
