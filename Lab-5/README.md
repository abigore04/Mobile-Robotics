# Line Following and Obstacle Avoidance using LEGO EV3 Mindstorms

**Course:** Mobile Robotics  
**Student:** Farid Ibadov  
**Instructor:** Alexzander Leonard Farral
**University:** ADA University  
**Date:** October 2025  
**Platform:** EV3Dev (Debian-based OS for LEGO EV3)  
**Language:** Python (EV3DEV2 library)

## Outline

- [Main Program](#main-program)
- [Objectives and Overview](#objectives-and-overview)
- [Main Code Explanation](#main-code-explanation)
  - [1. Library Imports](#1-library-imports)
  - [2. Hardware Setup](#2-hardware-setup)
  - [3. Constants and Tuning](#3-constants-and-tuning)
  - [4. Helper Functions](#4-helper-functions)
    - [4.1 Smooth Stopping](#41-smooth-stopping)
    - [4.2 Obstacle (Box) Dealing](#42-obstacle-box-dealing)
    - [4.3 Last Dead End Dealing](#43-last-dead-end-dealing)
    - [4.5 Moving Bit Straight in the Beginning](#45-moving-bit-straight-in-the-beginning)
  - [5. Main Loop](#5-main-loop)
- [Summary](#summary)
- [YouTube Link](#youtube-link)


---

## Main Program

```python
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
```

---

## Objectives and Overview

For this lab, the EV3 robot was expected to follow a main black line, avoiding getting lost in dead ends and to avoid collisions with obstacles. The logic is similar to the previous labs; however, now we have just single-colored tape, with no end indication, so, the robot is expected to understand where it needs to stop as well.

 Here is the track itself:
 <img width="2560" height="1440" alt="Pasted image 20251102151630" src="https://github.com/user-attachments/assets/f6d51c24-c15f-4fd7-be23-f0bbd0df7a8a" />


As we can see, we have 4 dead ends to deal with. Additionally, we can observe blue rectangle drew on the track - place where the box will be placed. 

So, there are many approaches how to accomplish this lab. Some of those that came to my mind:
- Use the similar to the line visualization behavior that was used in one of my previous labs, where the first run can be dedicated to the learning of track, and the second run will be, basically, following the most optimal (shortest) route, selected by the appropriate algorithm (such as Dijkstra).
- Use line visualization and map learning, but instead of using the algorithm, state the path for the robot manually.
- Design a specific hardware setup that will work fine for the this specific track and use the surrounding environment to help navigate efficiently.

Due to the lack of time, I have decided to stick to the last option. For that, I have introduced additional sensors: +1 color sensor - on the left of the existing one (to make left turns, since those are in abundance), -1 gyro (unplugged gyro for global orientation, used in previous lab, and left only one - for precise turns), +1 ultrasonic sensor mounted on the top front of the robot (to recognize obstacle in a form of a box, and to deal with the last dead-end)

So, in end my EV3 looked like this:
<img width="2560" height="1440" alt="Pasted image 20251102152713" src="https://github.com/user-attachments/assets/46e69ca9-6d1e-422a-a6c7-cd4f583a36ed" />


Once again, the robot should follow the line, by dealing with dead ends, stop when the obstacle (box) hinders further movements and wait till the obstacle is removed, then continue moving on, and, finally, stop at the very right end of the track.

Later, I thought to add a bit of creativity and mounted the middle motor on the right  of the robot with a long stick attached to the motor's shaft. Now instead of waiting till the box is removed by someone else, the robot can basically slap and knock it off by itself.

<img width="528" height="420" alt="Pasted image 20251102153215" src="https://github.com/user-attachments/assets/e4f89f17-1107-4cc9-bc36-ea0c71dfa9cd" />

- *epic.*

Regarding environmental observation, I have programmed the box counter, so after the robot removes the box and continues, it increases the value of this counter, and later, it will look for an obstacle that is closer that 25 cm - to detect the glass window near the last dead-end and to perform a "short-cut". This part of program can be disabled at any time, to follow without "cheating", but then we should introduce another counter, so the robot could oscillate (when getting lost and hunting for a line) more that 180 degrees **just once** - to handle the last dead end (other dead ends are handled with the left color sensor), so the next time, its maximum oscillation amplitude is less than 180 - to stop at the end of the track and do not return back.

## Main Code Explanation

As in previous EV3-labs, here, as well, I have designed the code on the basis of my original code (from the very first EV3 lab). This is handy, since it helps to see how basic code can grow and build up additional functionality, increasing its complexity. 

As it happens during any learning journey, the more you work with something, the more you start to feel how it works. It this lab similarly, I have understood that the robot follows line much better and smother when the target value is in the between of 2 values' thresholds. Additionally, for the first time, I have decided to use a function for a smooth stop, instead of just turning the motors off - this helped to remove jumps occurring at the back of the robot. All those, polished and refined movements of the robot a bit.

*Note: as mentioned before, this code is built upon previous code, so I will focus just on new parts.* 

### 1. Library Imports

```python
from ev3dev2.motor import MoveTank, MediumMotor, SpeedPercent, OUTPUT_B, OUTPUT_C, OUTPUT_D
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3, INPUT_4
from ev3dev2.sensor.lego import UltrasonicSensor, GyroSensor, ColorSensor
from ev3dev2.sound import Sound
from time import sleep, time
```

- Imported **medium motor** for robotic arm, **ultrasonic sensor** for obstacle detection, sound for beeps while detecting box and finishing the program (not finding a line). 

### 2. Hardware Setup

```python
mT =MoveTank(OUTPUT_B, OUTPUT_C)
mArm =MediumMotor(OUTPUT_D)
uS =UltrasonicSensor(INPUT_1)
gS =GyroSensor(INPUT_2)
cS =ColorSensor(INPUT_3) 
lS =ColorSensor(INPUT_4)
spk = Sound()

cS.mode= 'COL-REFLECT'
lS.mode = 'COL-REFLECT'
```

- `mArm` for a medium motor (attached to the port D), `uS` for ultrasonic sensor (attached to the port 1), and `spk` for sound. Both color sensors `cS` (central - existing one) and `lS` (left - newly introduced) are in Reflectance Mode.

### 3. Constants and Tuning

```python
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
```

- Set target `TARG_LITE` to 60 for more smooth line following. Lowered `kP` to avoid overshooting and increased `kD` to reduce jiggling.
- `STOP_CM` set to 5 cm - to stop in front of a box.
- `SCAN_THRESHOLD` set to 25 cm - to detect glass window and perform a 180-spin to avoid the last dead end.
- Introduced `box_counter` for window detection after some value of it.

### 4. Helper Functions

#### 4.1 Smooth Stopping

```python
def smooth_stop(decay_time=0.005, step=0.0025):
    spd =30
    while spd >0:
        mT.on(SpeedPercent(spd), SpeedPercent(spd))
        spd -= 30*step/decay_time
        sleep(step)
    mT.off(brake=False)
    sleep(0.05)
```

Starting from a speed of 30, with the decay time of 0.005 and step of 0.0025, the robot makes a smooth stop by decreasing the `spd` value with each iteration, for instance:
- 1st iteration: 
$$spd=30-30\cdot\frac{0.0025}{0.005} = 30\cdot0.5=15$$
- 2nd iteration: 
$$spd=15-30\cdot\frac{0.0025}{0.005}=15-15=0$$

- So we stop in 2 iterations - almost instantly, but smoothly.

#### 4.2 Obstacle (Box) Dealing

```python
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
```

- Stop when box is detected. 
- Beep for 3 seconds.
- Run middle motor at speed 90 (clockwise) for 0.3 seconds
- Run middle motor at speed -90 (counterclockwise) for 0.3 seconds
- Increase box counter by 1.

#### 4.3 Last Dead End Dealing

```python
def spin_180():
    reset_gyro()
    mT.on(SpeedPercent(35), SpeedPercent(-35))
    while abs(gS.angle)< 180:
        sleep(0.005)
    mT.off(brake=True)
    sleep(0.2)
```

- spin 180 degrees using gyro at speed 35 on both motors.

#### 4.4 Announcing the End of the Program

```python
def end_alarm():
    start =time()
    while time()-start < 2:
        spk.tone(400, 100)
        sleep(0.2)
```

#### 4.5 Moving bit Straight in the Beginning

If we place our robot with both of its color sensors above the black start-cross, it can deviate left initially (since we told that the left sensor tells the robot to move to the left by some degree). Therefore, we tell the robot to move straight blindly for 0.2 seconds at speed of 30 to avoid an undesired turn. 

```python
sleep(0.3)
start_time= time()
while time()-start_time< 0.2:
    mT.on(SpeedPercent(30), SpeedPercent(30))
    sleep(0.01)
smooth_stop()
```

### 5. Main Loop

- Explained using comments in a code block.

```python
try:
    while True:
	    # Stop when distance is less than or equal to STOP_CM (5)
	    # Handle box using obstacle_alarm()
	    # reset last error and last turn value
        if uS.distance_centimeters <=STOP_CM:
            obstacle_alarm()
            last_e =0.0
            last_turning_val = 0.0
            continue
		
		# only if obstacle_alarm() is run at least once, and
		# ultrasonic reads <= 25, spin 180, 
		# reset last error and last turn val 
        if box_counter>=1 and uS.distance_centimeters<=SCAN_THRESHOLD:
            spin_180()
            last_e =0.0
            last_turning_val =0.0
            continue

        cur_l = cS.reflected_light_intensity
        cur_side_l = lS.reflected_light_intensity
        lite_hist.pop(0); lite_hist.append(cur_l)
        avg_lite = sum(lite_hist)/len(lite_hist)

		# when left color sensor detects values <= 12,
		# turn to left at speed 35 for 15 degrees
        if cur_side_l<=12:
            smooth_stop()
            reset_gyro()
            mT.on(SpeedPercent(-35), SpeedPercent(35))
            while abs(gS.angle) < 15:
                sleep(0.005)
            smooth_stop()
            continue

		# only when both color sensor went off the line, hunt for line
		# if not found, run end_alarm()
		# reset last error and last turn val
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
```

- As it can be observed, the code is very similar to the ones we saw it previous labs, yet efficient.

## Summary

In this lab EV3 robot was programmed and equipped with specific sensors to follow a uniform black line, deal with four dead ends and an obstacle placed along the line. Unlike previous labs, where we had the second grey cross-line for a stop, in this lab we had no stop-line indicator, so the robot had to understand where to stop by itself. 

Specifically for this track, a combination of an additional color sensor on the left and an ultrasonic sensor at the top front became very handy. Since we had left-turns in the majority, the left sensor helped to avoid the first three dead ends and to turn where the path curved to the left. The ultrasonic sensor helped to stop near a box, which the robot successfully knocked off using an arm attached to the middle motor's shaft. After that, the program was searching for another obstacle - a glass window, to deal with the last dead end and return back where the left sensor identified the left turn - continuation of the path, and the robot could reach the end. At the end, the robot oscillated with an amplitude of less than 180, and understood that there are no turns left, and this is probably the end, and then beeped and finalized the program  

## YouTube Link

[Lab 5 EV3 Line Following Obstacle and Dead-ends Avoidance](https://youtu.be/5Uqb8tn80kU)
