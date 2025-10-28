# Basic Line Following using LEGO EV3 Mindstorms  

**Course:** Mobile Robotics  
**Student:** Farid Ibadov  
**Instructor:** Alexzander Leonard Farral
**University:** ADA University  
**Date:** October 2025  
**Platform:** EV3Dev (Debian-based OS for LEGO EV3)  
**Language:** Python (EV3DEV2 library)
## Overall Structure

- [Whole Program](#whole-program)
- [Objective and Overview](#objective-and-overview)
- [Code Explanation](#code-explanation)
  - [1. Library Imports](#1-library-imports)
  - [2. Hardware Setup](#2-hardware-setup)
  - [3. Constants and Tuning](#3-constants-and-tuning)
  - [4. Helper Functions](#4-helper-functions)
    - [4.1 Resetting the Gyro](#41-resetting-the-gyro)
    - [4.2 Performing 360-Spin](#42-performing-360-spin)
    - [4.3 Avoiding the Second Read of the Same Cross Line](#43-avoiding-the-second-read-of-the-same-cross-line)
    - [4.4 General Turning Behavior using Gyro Sensor](#44-general-turning-behavior-using-gyro-sensor)
    - [4.5 Searching Method when Got Lost](#45-searching-method-when-got-lost)
  - [5. Main Loop](#5-main-loop)
    - [5.1 Reading sensors, smoothing light, getting time](#51-reading-sensors-smoothing-light-getting-time)
    - [5.2 Lost line detection, search and recovery](#52-lost-line-detection-search-and-recovery)
    - [5.3 Cross-line detection with signal validation](#53-cross-line-detection-with-signal-validation)
      - [5.3.1 Cross-line Behavior](#531-cross-line-behavior)
    - [5.4 Regular line following using PD Control](#54-regular-line-following-using-pd-control)
- [Summary](#summary)


---

## Whole Program

```python title:"wheel based Lab-2 Code"
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
```

---
## Objective and Overview

Simply, this program, once runt on a LEGO EV3 Brick, aims to enable the robot to autonomously follow initially preset line with varying light reflection intensity - due to imperfections in tape material, and perform various maneuvers once specifically-colored stripes encountered. 

Here are the track and instructions:
<img width="1066" height="692" alt="Pasted image 20251014192612" src="https://github.com/user-attachments/assets/5b70da86-c359-47ad-b102-6f365470c1db" />


As illustrated in the attached image, the robot is expected to follow the **main line**, do **360-spin** once it encounters the first grey line, and **stop** as soon as it reaches the second green line.

Lego EV3 Brick in this lab was equipped with a pre-configured Micro-SD cart (with **EV3Dev operating system**, a Debian-based Linux image) enabling us to program the brick using Python. Sensors that were used in this lab were LEGO's **RGB color sensor** to distinguish the see the track and **Gyro sensor** to make precise turns and perform 360-degree spin.

In the specific code implementation, the line-searching method was programmed by using *Proportionate-Derivative* (aka **PD**) *feedback control* along with oscillations with increasing amplitude, once the robot goes off the track and stops (aka *Active Perception search method*).

It is important to take a look at the appearance of the brick. In my case, I have a brick that looks something like that:

<img width="1180" height="664" alt="Pasted image 20251025234032" src="https://github.com/user-attachments/assets/503857cc-6953-462b-b723-fc2fa4c63880" />

*Note: in the following lab, only 1 gyro was used, the second gyro was intended for the next lab.*

It is also important to mention the **distance from the ground** of the color sensor, which was around **4 mm**, and **lighting** in the lab which was was fully on.

<img width="906" height="980" alt="Pasted image 20251025234207" src="https://github.com/user-attachments/assets/35113405-d363-4d27-93a7-4651c83acb59" />

---

## Code Explanation

### 1. Library Imports

- First we need to import necessary library and initialize motors and sensors to basically be able to control the EV3 hardware.

```python
from ev3dev2.motor import MoveTank, SpeedPercent, OUTPUT_B, OUTPUT_C
from ev3dev2.sensor import INPUT_2, INPUT_3
from ev3dev2.sensor.lego import ColorSensor, GyroSensor
from time import sleep, time
```

- It will be clear later where all they were used and how they work.

---
### 2. Hardware Setup

```python
mT =MoveTank(OUTPUT_B, OUTPUT_C)
cS =ColorSensor(INPUT_3)
cS.mode = 'COL-REFLECT'
gS =GyroSensor(INPUT_2)
```

by `cS.mode` we specify the mode through which our Color Sensor will operate. I have selected **Color Reflect** mode, which will return the color intensity in the integer values from 0 (darkest) to 100 (brightest). 
Since our track is designed with color-inconsistent tape, and other factors, such as shadows, can cause in floating measures, *Regular Color Mode* is not suitable (since colors are not consistent), as well as *Ambient Light Intensity Mode*, which does not emit any light and is used just for plain light detection.
**Color Reflect** with a **range** of values works the best in this case. Range can be simply determined by following the bot manually along the tape with Color Sensor values displayed on a screen (pressing to the Watch Values) section.

So, basically:
- **`mT`** - controls the **movement** of the brick
- **`cS`** - detects the **brightness** of a surface
- **`gS`**  - tracks rotational **angle**

---
### 3. Constants and Tuning

Here we introduce numeric constants that set desired **thresholds**, **speeds of motion**, and **control parameters**. This is the part, where constant adjustments are required the most, to achieve desired behavior. 
These values were obtained experimentally, **by trial and error**, while observing the sensor readings on the track. 
It is important to mention that the robot may act unexpectedly once the room's lightning is not the same as the lab's one, if shadow is casted on the track, or the distance between the color sensor and table is changed; however, values always can be adjusted for a particular environment.

```python
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
```

- Threshold Values for **Color Sensor** (all are in %, from 0 to 100%, since 'COL-REFLECT'):
	- `LINE_MAX =24` - maximum value on green (main, dark) tape.
	- `CROSS_MIN,CROSS_MAX =24,40` = values' range for grey crossing lines.
	- `WHITE_MIN =70` = minimum value when the robot understands that it got lost.
- **Motors** and **PD control**:
	- `TARG_LITE =9` - is optimal and desired brightness to follow the main tape; around this value the controller will keep *average reflectance*.
	- `BASE_M_SPD =20` - base motor speed (in %) for both wheels - this one is different from the one that appears during the corrections.
	- `kP =0.3` - this is so-called "**Proportional gain**". It mainly reacts to the brightness error in our case - "*how far the robot is from the main line*". The importance of `kP` will be noticeable later, as we will come to the appropriate equation for motion control.
	- `kD =3.0` - known as "**Derivative gain**". It mainly controls the oscillations by reacting how quickly error changes. We can say that it predicts the robot's motion. It helps to prevent the robot from going "too far", while correcting its path. As with `kP`, its importance will be visible later.
		- Here rises the question "*why have we used the PD control instead of PID control?*". The answer is that for our goal, the regular line following, the Integral part is not so important, since the Light Sensor values change frequently when the robot moves and there exist some jiggles, we are left with no solid error to fix, and if we have used it, it would keep adding small random changes and would make the robot to oscillate even more, with a prospect to completely lose the line.
	- `MAX_T =24` - to avoid oversteering. Is used in clamping function to limit the value of motor power from PD Control between boundaries of -24 and 24. It is a safety measure.
	- `SLEW_RATE =6` - provides smoother motion by limiting rate of change in turn command between the loops. Basically, it gradually increases the turn by the value specified. Will be more clear later.
	- `DT =0.025` = delay for the PD control loop. It determines update frequency of the loop (how fast corrections take place). Here set as 0.025 seconds, which corresponds to 40 Hz (so the loop updates 40 times per second).
- **Runtime Variables** Initialization.
	- `lite_hist =[cs.reflected_light_intensity]*3` – list that stores last 3 light-reflected values to obtain one average. by doing so, we filter noise and avoid abrupt value changes.
	- `cross_count` – to count the number of cross-stripes detected (our goal is 2, 1st is 360-spin, 2nd - complete stop). 
	- `x_streak` – counts consecutive grey readings (used to avoid false detection on the edge of main tape, where the sensor can read cross-stipe's value).
	- `next_x_time` – is the timestamp to prevent multiple detections of the same cross (like when robot did 360 and the sensor casts the light on the same cross-tape for the second time).
	- `last_e` – is the last loop’s brightness error. It is used for the derivative term.
	- `last_turning_val` – turn value that was applied lastly. It is used for slew-rate limiting.

*Note: the role of each constant and variable introduce here will be understood more clearly later.* 

### 4. Helper Functions

Here, we define functions that will be used in the main loop. Each function here performs specific action, such as turning, searching, resetting. 

#### 4.1 Resetting the Gyro

- We need to reset the values of the gyro since it may initially come with some deviations. It is *always called before any turning action* due to, so-called, "Gyro Drift". Resetting gyro values before every its use is important if we want to achieve accuracy.

```python
def reset_gyro():
    gS.mode= 'GYRO-RATE';sleep(0.15)
    gS.mode= 'GYRO-ANG';sleep(0.15)
```

- By `'GYRO-RATE'` mode we reset internal measurements of drifts and reinitialize the rotation rate of the sensor. This mode essentially measures the angular velocity (degrees/s). When the robot turns, it shows some values (+n degrees/s clockwise or -n degrees/s counterclockwise). 
- By `'GYRO-ANGLE'` we measure the angles by which gyro turns, starting from 0 degrees. (+n degrees clockwise or -n degrees counterclockwise). 
- We introduce short delays of 0.15 seconds to let the sensor to stabilize.
- As a result, `gs.angle` becomes 0 deg each time we call the function.

#### 4.2 Performing 360-Spin

```python
def spin_360_deg():
    reset_gyro()
    mT.on(SpeedPercent(33),SpeedPercent(-33))
    while abs(gS.angle)< 360:
        sleep(0.005)
    mT.off(brake=True)
```

- Function to perform 360-degree spin. We will call it when we encounter the first cross line. As it involves turning, we importantly reset the gyro through `reset_g()` function, that we have discussed earlier.
- Through `mT.on(SpeedPercent(33),SpeedPercent(-33))` function we perform basic "Tank Turn" maneuver, by rotating tracks in different directions at a speed of 33%.
- So, as long as the absolute value of our `gs.angle()` function is less than 360, we perform spin. As soon as it reaches 360, motors shut down by `mt.off(brake=True)` function.
- I introduced `sleep(0.005)` to make small 5-milllisec delays to avoid overloading the Brick's CPU. 

#### 4.3 Avoiding the Second Read of the Same Cross Line

```python
def drive_a_bit_frwrd():
    mT.on(SpeedPercent(19),SpeedPercent(19))
    sleep(0.6)
    mT.off(brake=True)
```

- This function is basically used after 360-spin to make sure that the robot does not read the cross-line tape for the second time and does not treat it as the second cross strip, where it can stop. It makes the robot to move forward at the speed of 19% of motors' power for 0.6 second to move its color sensor off the cross tape.

#### 4.4 General Turning Behavior using Gyro Sensor

```python
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
```

- This function is used for general turning in one place using gyro when searching for the main line.
- Passing Values:
	- `direction_sig` - defines direction of spin (1 for right, -1 for left turn).
	- `degs_to_oscill` - defines the maximum rotation angle in both directions.
	- `sp_to_oscill` - turning speed.

- `start_ang =gS.angle` - saves the current angle from gyro before performing the turn.

- `left_m_spd =sp_to_oscill * direction_sign` and `right_m_spd = -sp_to_oscill * direction_sign` - since we turn in place, we should oppose directions of motors' spins.

- Loop (explained using comments):
```python
# infinite loops that runs:
# 1. unless line is found, cs.reflected_light_intensity<= LINE_MAX is True
# 2. Turn limit is reached, abs(curr_ang -start_ang)>= degs_to_oscill, which return False
# we will use those return vaules in the next hunt_for_line() function

while True:
        curr_ang =gS.angle
        if cS.reflected_light_intensity<= LINE_MAX: #1st cond
            mT.off(brake=True);sleep(0.1)
            return True
        if abs(curr_ang -start_ang) >= degs_to_oscill: #2nd cond
            break
        sleep(0.005)
mt.off(brake=True)
return False # line isn't found regardless of the loop's work
```

#### 4.5 Searching Method when Got Lost

```python
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
```

- This function works with the previous almost as one whole and they are both aimed at the line search, once the robot's color sensor goes off the track.
- Essentially, the `hunt_for_line()` functions makes the robot to oscillate with the increasing amplitude. `angles = [30, 60, 90, 120, 150, 180]` array's values represent the degree by which the robot is expected to turn, using gyro. In my code, I specified the minimum value of 30 and the maximum value of 180 degrees. Those values can be adjusted if the robot accidentally returns back, detecting the tape section that was already passed.
- In `for` loop we can observe 3 if statements, first if makes the robot turn on the right by `angle` degrees, second - left by twice of `angle` (to cancel the right turn), and the third `if` is to reset the direction that the robot faces back to the original - ready to try the next angle.

#### 5. Main Loop

Now we come to the "beating heart" of our function - the main loop, where the essential control cycle of our robot is introduced. Main loop continuously reads the data from sensors, while deciding what behavior to perform based on that data fetched, like line following, searching, cross handling. It also controls motors' speeds in real time to make sure that everything runs correctly and the mission ends successfully.
It will be better to divide this part into separate snippets to explain it better (i will specify them as comments in the code block below):

```python
try:
    while True:
	    # 5.1 Reading sensors, smoothing light, getting time
        cur_l = cS.reflected_light_intensity
        lite_hist.pop(0); lite_hist.append(cur_l)
        avg_lite = sum(lite_hist)/len(lite_hist)
        now =time()

		# 5.2 Lost line detection, search and recovery
        if avg_lite>WHITE_MIN:
            mT.off(brake=True)
            if not hunt_for_line():
                break
            next_x_time = time() +0.3 
            last_e =0.0; last_turning_val =0.0
            continue

		# 5.3 cross-line detection with signal validation
        if now >= next_x_time and CROSS_MIN <= avg_lite<= CROSS_MAX:
            x_streak+=1
        else:
            x_streak=0

        if x_streak>=2:
            x_streak=0
            cross_count+=1
            mT.off(brake=True)
			
			# 5.3.1 Cross-line Behavior
            if cross_count==1:
                spin_360_deg()
                drive_a_bit_frwrd()
                next_x_time = time()+0.6
                last_e=0.0; last_turning_val=0.0
            elif cross_count==2:
                mT.off(brake=True)
                sleep(0.2)
                break

		# 5.4 Regular line following using PD Control
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
```

#### 5.1 Reading sensors, smoothing light, getting time

```python
cur_l = cS.reflected_light_intensity
lite_hist.pop(0); lite_hist.append(cur_l)
avg_lite = sum(lite_hist)/len(lite_hist)
now =time()
```

- `cur_l` is for current light reflected, `lite_hist.pop(0)` for removing old value, `lite_hist.append(cur_l)` for adding the new value, while holding 3 newest values, and by `avg_lite = sum(lite_hist)/len(lite_hist)` we calculate the average of 3 latest values to smooth the line.
- By `now =time()` we set current timestamp for cooldowns and signal validation.

#### 5.2 Lost line detection, search and recovery

```python
if avg_lite>WHITE_MIN:
	mT.off(brake=True)
	if not hunt_for_line():
		break
	next_x_time = time() +0.3 
	last_e =0.0; last_turning_val =0.0
	continue
```

- When `avg_lite` is more than the minimum value for white table detection, the robot stops using `mT.off(brake=True)` and understands that it went off the track.
- Then it launches `hunt_for_line()` function to reorient itself along the track.. If this fails, the program ends.
- When recovered successfully, short cooldown `next_x_time = time() +0.3` is set to avoid misinterpreting the tape as cross-line.
- Finally, we reset last error and last turning values for future hunts.

#### 5.3 cross-line detection with signal validation

```python
if now >= next_x_time and CROSS_MIN <= avg_lite<= CROSS_MAX:
	x_streak+=1
else:
	x_streak=0

if x_streak>=2:
	x_streak=0
	cross_count+=1
	mT.off(brake=True)
	
	# 5.3.1 Cross-line Behavior
	if cross_count==1:
		spin_360_deg()
		drive_a_bit_frwrd()
		next_x_time = time()+0.6
		last_e=0.0; last_turning_val=0.0
	elif cross_count==2:
		mT.off(brake=True)
		sleep(0.2)
		break
```

- Only if we passed the cooldown time interval and we detect the values of cross tape, we increase the `x_streak` counter by 1, otherwise, it stays 0.
- When `x_streak` is 2 or more, meaning it reached 2 consecutive detections for signal confirmation, we reset it to 0, and increase cross counter `cross_count` by 1, meaning, we have detected the real cross-line.
- After that, robot stops and performs the corresponding behavior, shown below.

##### 5.3.1 Cross-line Behavior

```python
if cross_count==1:
	spin_360_deg()
	drive_a_bit_frwrd()
	next_x_time = time()+0.6
	last_e=0.0; last_turning_val=0.0
elif cross_count==2:
	mT.off(brake=True)
	sleep(0.2)
	break
```

- Here everything is clear, when `cross_count = 1`, do 360; when `cross_count = 2`, stop. 

#### 5.4 Regular line following using PD Control

- Now we've come to the "cherry on top" of our program - **Control Strategy**.

```python
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
```

- `err =  TARG_LITE - avg_lite` 
		- by error we understand how far the readings from the sensor deviates from the desired value.
- `deriv =err-last_e`
		- using derivative term we detect how quickly the error changes.
- Having those two parameters, we can construct the equation for PD Control mechanism, achieving quick reaction and smooth corrections:
		- `turn_calculation = kP*err + kD*deriv`

```python
if turn_calculation>MAX_T: turn_calculation =MAX_T
if turn_calculation< -MAX_T: turn_calculation = -MAX_T
```
- Then, we just clamp the value obtained from `turn_calculation` between `Max_T` and `-Max_T` values, to avoid sudden destabilization.

```python
	min_s = last_turning_val- SLEW_RATE
	max_s = last_turning_val+ SLEW_RATE
	final_turning_value = max(min_s,min(turn_calculation, max_s))
```
- Next, we limit a slew rate, adjusting turning gradually, by avoiding jerks, so improving smoothness; and get our final turning value `final_turning_value`.

- Then, we update `last_turning_val` and`last_e` 


```python
	left = BASE_M_SPD +final_turning_value
	right = BASE_M_SPD -final_turning_value

	left = max(-100,min(left, 100))
	right = max(-100,min(right, 100))
	
	mT.on(SpeedPercent(left),SpeedPercent(right))
	sleep(DT)
```
- After that, we apply the turning gains to our base motors' speeds and clamp (limit) those values to avoid overflowing the maximum values for motors. Then, we just apply that to motors, followed by loop's update frequency.


## Summary

In this lab, I programmed a LEGO EV3 robot for autonomous dark green line following on a white table. I've used a PD control in order to achieve smooth locomotion without excessive jiggling or cases of drifting off the track too far. The robot continuously checked the data from a color sensor and a gyroscope to make turning decisions. It managed to spot gray stripes as line crossings and perform preset actions, such as a 360-spin or a stop. This experiment demonstrates the basics of how robots can control themselves using sensors.

Additionally, as I mentioned before, the code is suitable for all the morphological variations of EV3 brick that more or less resembles a car, just important to make sure that all the sensors and motors are connected to the right ports. Then, playing around with some variables it is possible to make running something like this:
<img width="2560" height="1920" alt="Pasted image 20251028155616" src="https://github.com/user-attachments/assets/208db91a-9d12-411c-9913-43cf55d518dd" />

Due to increased tension in transmission system, I had the `kP` and `kD` values increased to 1.1 and 13.0, correspondingly (comparing with 0.3 and 3.0 from the code for wheel-based). `BASE_M_SPD` was 25, although it is higher than in wheel-based code, the robot on tracks still moved slowly, again, due to increased tension.

You can observe the behavior of both of the robots if you will follow those links:
- Wheel-based:
https://youtu.be/NdSpatGHhNQ
- Track-based:
https://youtu.be/AWknjbEQrwQ


