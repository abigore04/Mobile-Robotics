# **Tic-Tac-Toe Using Yahboom Transbot**  

**Course:** Mobile Robotics  
**Students:** Farid Ibadov & Said Ahadov.
**Instructor:** Alexzander Leonard Farral.
**University:** ADA University  
**Date:** January 2026  
**Platform:** Raspberry Pi 4 + Yahboom Transbot hardware, ROS Melodic
**Language:** Python (rospy, OpenCV)

---
# Overall Structure

- [0. Executive Summary & Quick Overview](#0-executive-summary--quick-overview)
- [1. System Overview](#1-system-overview)
	- [1.1 Main Purpose](#11-main-purpose)
	- [1.2 System Components](#12-system-components)
		- [1.2.1 Physical and Hardware Components](#121-physical-and-hardware-components)
		- [1.2.2 Software Components](#122-software-components)
	- [1.3 Scripts, ROS Nodes and Their Jobs](#13-scripts-ros-nodes-and-their-jobs)
	- [1.4 Communication. Topics and Services](#14-communication-topics-and-services)
		- [1.4.1 Board State](#141-board-state)
		- [1.4.2 Motion Service](#142-motion-service)
	- [1.5 Nodes - Bigger Picture](#15-nodes---bigger-picture)
- [2. Mechanical & Hardware Design](#2-mechanical--hardware-design)
	- [2.1 Platform](#21-platform)
	- [2.2 Robotic Arm and DOF Configuration.](#22-robotic-arm-and-dof-configuration)
	- [2.3 Arm Extension Modules](#23-arm-extension-modules)
	- [2.4 End-Effector](#24-end-effector)
	- [2.5 Boards and Physical Layout](#25-boards-and-physical-layout)
	- [2.6 Computing and Hardware Reliability](#26-computing-and-hardware-reliability)
	- [2.7 Hardware List](#27-hardware-list)
- [3. Calibration](#3-calibration)
	- [3.1 Calibrating Boards](#31-calibrating-boards)
	- [3.2 Calibrating Robot Base](#32-calibrating-robot-base)
	- [3.3 Calibrating Arm Base](#33-calibrating-arm-base)
- [4. Perception System (Vision)](#4-perception-system-vision)
	- [4.1 Purpose and role in the overall system](#41-purpose-and-role-in-the-overall-system)
	- [4.2 ROS interface (what the node subscribes/publishes)](#42-ros-interface-what-the-node-subscribespublishes)
	- [4.3 High-level pipeline (image → board state)](#43-high-level-pipeline-image--board-state)
	- [4.4 Board detection (code explanation)](#44-board-detection-code-explanation)
	- [4.5 Picture Warp](#45-picture-warp)
	- [4.6 Cell extraction](#46-cell-extraction)
	- [4.7 HSV thresholding](#47-hsv-thresholding)
- [5. Motion & Manipulation](#5-motion--manipulation)
	- [5.1 Main Purpose](#51-main-purpose)
	- [5.2 Final Motion Concept](#52-final-motion-concept)
	- [5.3 Substitute for Inverse Kinematics](#53-substitute-for-inverse-kinematics)
	- [5.4 Home Pose](#54-home-pose)
	- [5.5 The Service Interface](#55-the-service-interface)
	- [5.6 Safety Limits](#56-safety-limits)
	- [5.7 Key Idea - Ordered Actuation](#57-key-idea---ordered-actuation)
	- [5.8 Smooth pan](#58-smooth-pan)
	- [5.9 "Atomic" Motion Primitive](#59-atomic-motion-primitive)
	- [5.10 Storing a "state" Dictionary](#510-storing-a-state-dictionary)
	- [5.11 Execution Logic](#511-execution-logic)
	- [5.12 What is out of Scope of This Node](#512-what-is-out-of-scope-of-this-node)
- [6. Game Logic & Decision-Making](#6-game-logic--decision-making)
	- [6.1 Inputs and Outputs](#61-inputs-and-outputs)
	- [6.2 Board Representation made Internally](#62-board-representation-made-internally)
	- [6.3 Turn Management A/P Player support](#63-turn-management-ap-player-support)
	- [6.4 Interactive Mode Logic](#64-interactive-mode-logic)
	- [6.5 Minimax](#65-minimax)
	- [6.6 Autonomous Mode Logic](#66-autonomous-mode-logic)
	- [6.7 Storage Pieces](#67-storage-pieces)
	- [6.8 Failure Handling & Timeouts](#68-failure-handling--timeouts)
- [7. Discussion and Conclusion](#7-discussion-and-conclusion)
	- [Discussion](#discussion)
	- [Conclusion](#conclusion)
- [8. References](#8-references)

---

# 0. Executive Summary & Quick Overview

In this introductory section, a concise overview of the project objectives and a detailed walkthrough of the project's improvements and weaknesses are presented. The purpose of this report is to introduce the reader to the project's main goal, take a tour of the entire path taken to achieve the this goal, and engage with a fascinating story. Further and more intricate details will be observed in the corresponding parts of the report. Therefore, *this section may be skipped if you wish to see more technical details and results.*

In this project, we have implemented and integrated the basic tic-tac-toe game functionality within a mobile robot by using all the essentials of robotics and coding, including **ROS** ecosystem, **PID control**, **Inverse Kinematics**, **Minimax algorithm**, and **Computer Vision**, by gluing all together with creativity and critical thinking, finding the most optimal and rational ways. The platform we used is the **Yahboom Transbot robot**, which initially came with a powerful NVIDIA Jetson Nano computer, and which was eventually replaced with a simple and robust **Raspberry Pi 4**. The reason for the swap was the instability that was caused by either factory-assembly issues or boot configuration errors, resulting in constant stalls in boot and even in total data corruption. Raspi (aka Raspberry Pi) was the system more familiar to us, and since the project itself was not so power-requiring, this was the best option to stick to.

Overall, the project consists of two essential parts - **interactive** and **autonomous** game modes. 
	- In **interactive mode**, the robot was expected to play tic-tac-toe against a human being by physically interacting with the environment by manipulating 3D-printed figures with the 3-DOF (Degrees of Freedom) robotic arm to pick up from one board (storage board) and place on another board (playground board). After checking the state of the playground using Computer Vision techniques and passing the parameters to the Minimax - recursive algorithm designed to opt for the most optimal move to lead to either win or draw, robot reaches the storage board with all the figures *placed in order* (in first cells - Xs, in remaining - Os) to grab desired one and places it in the cell of the playground board which the algorithm has chosen to be the best. In a nutshell, in an interactive mode, player chooses to be an *active player* (to make the move first with an X figure) or a *passive player* (to make the move using an O figure, after the robot makes its move with an X). In either scenario, there are two outcomes - ether robot **wins**, or it is a **draw** - no one wins. 
	- On the counterpart, in **autonomous mode**, where the robot was expected to make moves *sequentially* - first placing Xs and then Os, instead of Minimax algorithm we have decided to make moves **random** to increase the interest and add some gambling spice - so the outcome is unpredictable (although the probability for X to win is higher - simply since the number of X figures exceeds the number of Os and X comes first.)

One of the biggest difficulties faced was the **tracks**. As the robotic arm in its initial configuration had only 3 DOFs - one servo for the gripper (*s9*) and two for the arm bending in one plane (*s8* connecting forearm and shoulder, and *s7* connecting shoulder and base), essentially the robotic arm could only reach the objects located precisely in front of the robot itself. The only way to reach the figures located to the left or right was to turn the base of the robot using the tracks. The robot turns its base by spinning the left and right tracks in opposite directions. Although we used *Gyro* data and *PID* control to reach precise angular turns, the varying friction between the surface on which the robot stays and the face of the tracks resulted in small drifts to the back or forward direction the robot faces. Though in a couple of moves it won't cause any significant errors, in continuous games or in autonomous mode, where it had to make 9 moves, those errors could build up, resulting in a complete failure. Therefore, we came up with a clever and simple solution - to mount the arm on the **pan servo** located on the very top of the robot, which was initially designed for the cameras. By doing so, we managed to achieve an absolute precision in the frame of our project, so that even after 10 games played, the robot could reach each cell accurately. 

However, with this kind of modification, we faced another, **arm issue** - the arm's shoulder and forearm were **too short** to let the gripper reach the cell - *14.5 cm* between the tip of the gripper and the s8 servo, and *8 cm* between the s8 and the s7 servos. So, using Lego sticks and imitating the robotic arm, we have come to the idea of 3D-printing extension modules, **4.5 cm**-long for the forearm and **11 cm** for the shoulder, so the total length of our arm from the tip of the end-effector (gripper) and s7 servo became ***38 cm***. After doing all these, the robot could accurately reach each cell with ease. 

![Pasted image](Pasted%20image%2020260104040019.png)

Another thing that we have changed is the **camera location**. From the factory, it was designed so that the camera mounts on the top of the pan servo in the middle of the robot. In the previous configuration, where we still tried to use the tracks for turns, we have 3D-printed the mounting module to place the Orbbec Astra Depth camera on the top of the forearm. Though the camera is heavy, servos could manage to handle that weight. By doing so, we thought to have some constant servo angle settings at which the matrix of the camera looked straight at the board, capturing all the cells. This could work perfectly unless we had precision issues with tracks. So in our later modification where the arm's base position was altered and the arm itself was extended, sticking to this configuration was risky and arduous due to 3 reasons: *first* - the length of the cable was not sufficient for the extended arm, however this could be mitigated by using USB cable extension, *second* - the weight of the camera could create so much pressure on the base that could snap off the bearing of the pan servo or break the connections of 3D-printed extensions, *third* - servo gears could not resist increased pressure and by trying to compensate the angular moment created by increased weight, the motors of servos could overheat, melting all the components and breaking the servos completely. So the only reasonable option was to *increase the length of the camera cable* using a USB extension and *attach the camera to the tripod* set aside, so the camera looks at the playground from top to bottom.

The camera is a third-party device, using which the algorithm keeps itself updated in terms of the state of the playground to notice all the changes using computer vision - particularly HSV thresholding, since all the figures and the board itself have *different colors* (black for X, white for O, and blue for the borders of the board). Cells are distinguished using edge and corner detection. Empty cells have the brownish color of the cardboard.

![[Pasted image 20260104040551.png]]

Another difficulty faced was the method of implementing the **grabbing and releasing**. We did not overcomplicate things and decided to use the physical properties of the magnets. As a result, we have purchased a bunch of neodymium magnets - small ones having a diameter of 4 mm and height of 3 mm to fit perfectly into the holes of figures which were designed in Autodesk Fusion (3D modeling software) and large magnets with 10 mm in diameter and 2 mm in thickness to place them under the cardboard sheets within the playground board cells. One of the large magnets was attached to the gripper itself, keeping a constant servo angle to hold the magnet firmly (although the magnet stuck to one lip of the gripper to avoid falling during the calibration process). An important part here was to make sure that the *polarities of the magnets do not conflict* so the gripper's magnet could easily grab the figure from the storage board and place it into the cell of the playground board where the figure snaps to the cardboard surface, since the small magnets embedded in figures are exposed more to the playground magnets than to the grippers magnet. With this neat design, grab and release functionality was implemented almost perfectly, though sometimes, especially with O figures, the borders of the *storage board snap the figure off the gripper* - the probability of this happening was reduced to a minimum with correct gripper magnet placement.

![[Pasted image 20260104040224.png]]

Talking about the **calibration**, the most suitable way for us to calibrate the robot with respect to the boards was to first place each board perfectly parallel to the edge of the table (or any straight-edge object, like a regular box). The correct placement of boards against each other was established using the regular *ruler* placed just between them, which has a **2 cm** thickness. The base calibration was established by placing the tool with an inductive coil found in the lab, which has around **4 cm** of thickness. Why 4 cm? This is the most optimal distance for the end-effector to reach the closest cell without colliding with the border of the board and the farthest - without exerting too much pressure on the base. Finally, to calibrate the base of the robotic arm with the center of the playground, before the game, we launch a short script to stick out the arm forward and down to **{"s7": 123, "s8": 180, "s9": 107}** so that the lips of the gripper align with the borders of the center cell.

![[Pasted image 20260104035636.png]]

Another great issue encountered was the so-called **servo conflict**. Sometimes servos appeared to override each other when one servo suddenly locks, or another starts to move at increased speed. This could be avoided using various techniques, like calculating the angle difference between the starting angle of the servo and the target angle to rotate to for each servo and set similar rate of turn for all of them to achieve synchronous movement; however, we decided to keep everything simple and make some order of actuation, for instance, when moving from home position to grab or release position, the robot first turns with *pan* servo, then with *s8* (forearm) and finally with *s7* (shoulder). This helped to avoid servo conflict and the possibility of the end-effector colliding with a surface the robot is standing on or with the borders of the boards.

With all the following techniques and modifications, after a bunch of testing, we have achieved promising results. After **20 games played**, the success of **board detections** (when the game was not interrupted with the board being undetected) was *95%* - 1 fault to 20 games played. **Cell classification** accuracy when the board was detected was *90%* (2 failures). The **figure manipulation** success rate was *85%* (only 3 failures). **Average time** for the robot to make the move, counting from the moment when the human placed the figure up to the moment when the robotic arm returned to the home position, was *28.5 seconds*. 

So, the main limitations left are the *difficulties in cell classification*, which were mitigated by constant lighting in the room and avoiding the shadows being cast on the playground, *grabbing the figure*, which was partially fixed with optimal gripper magnet placement, and *hardware overheating*, which was partially mitigated after encapsulating the Raspberry Pi into a case with active air cooling using one fan.

All the changes in the robot's physical configuration made along the path of the project can be observed in the following images, separated into stages: 

#### **Stage 1** (Factory configuration, Jetson Nano, 3-DOF arm):

![[aaaa.jpg]]

#### **Stage 2** (Raspberry Pi, camera placed on 3-DOF arm):

![[aaa.jpg]]

#### **Stage 3** (Raspberry Pi with active cooling, camera set aside, extended 4-DOF mounted on the top of the robot):

![[Pasted image 20260104035341.png]]

---

# 1. System Overview

Complete system architecture and how various subsystems communicate with each other are demonstrated and explained in this section. 
## 1.1 Main Purpose

Basically, the project itself is a **modular system** with the main configuration consisting of 4-DOF robotic arm manipulator mounted on the top of the mobile track-based platform capable of performing repetitive tasks until the goal is reached. Simple system pipeline can be visualized like this:

![[Pasted image 20260107004257.png]]

The main idea is to make the system **structural** by dividing responsibilities - when each component performs separately and does one job. Therefore, the **ROS** system was used to act as the wiring, making sure that a firm communication is being constantly established. Additionally, this kind of system setup makes the process of testing, debugging more convenient, while also being quite flexible, providing the opportunities to expand the project further.

## 1.2 System Components

System components can be understood as building blocks, each playing a significant role in overall system performance. With a lack of particular components, the system can suffer from being overcomplicated or even incomplete. Those components can be separated into two big groups: **Physical/Hardware** and **Software** components.

### 1.2.1 Physical and Hardware Components
#### Double-board setup
The most practical, yet efficient way of realizing the convenience of grabbing and releasing mechanism was to ensure **staticity**. We achieved this by using two separate 3x3-matrix boards - one playing a role of a *storage*, another - *playground*. By keeping all the figures in the storage board in pre-defined order and hardcoding this order, the system can reach each figure with high accuracy, making sure that correct figure has been picked up. Although this could be done using vision, our setup unloads the system dramatically.

![[Pasted image 20260107012251.png]]

#### Manipulation Pattern
Manipulation pattern has been realized quite simply as repeatable **" HOME -> target -> HOME -> target -> HOME "** pattern. Robotic arm has a particular predefined home position with preset s8 and s7 servo angles (for pan the home is defined separately and s9 stays fixed to hold the magnet) to which it tends to return after each maneuver. 
So, suppose, if it was commanded to pick up the some figure from the storage board and place in playground board. Arm will:
	1. face the first target cell by turning the entire arm using pan servo in that direction
	2. reach that cell using predefined pattern by extracting the arm
	3. retract it to home pose
	4. turn to face the second target cell
	5. extract to reach target
	6. retract to home pose
	7. return to some home pose for pan

at which it will rest till the program force it to perform another round. Why to retract after extracting? Simply to reduce the load on the base of the arm, created by increased leverage due to huge weight created by extended arm.

#### Perception
Orbbec Astra Depth camera which is mounted on top of the tripod put aside is used in our setup, providing constant RGB stream which is used by the vision pipeline. This camera has been chosen due to the availability of USB connection in which case the extension adapted can be used to freely move the camera in a wide range far from the robot. Having a wide field of view is another advantage of that camera, providing 60 degrees vertical and 49.5 degrees horizontal view it allows to get closer to the board without losing its corners in the view and winning in resolution resulted in closer proximity allowing high quality vision. 

#### Others
- *Other Mechanical features are discussed in Section 2: "Mechanical & Hardware Design"*

### 1.2.2 Software Components

Software components include subsystems each doing a particular job while communicating with each other to achieve the system's goals. Subsystems:
	- **Vision**: publishes the playground board state as a 3 by 3 matrix.
	- **Game**: feeds on board state provided by vision subsystem and determines the next move (minimax for interactive, random for autonomous) 
	- **Motion**: performs physical motion through robotic arm after getting commands through a service interface.

## 1.3 Scripts, ROS Nodes and Their Jobs

So the system in its final shape can be conceived of being constructed of the following executable scripts and nodes:

#### Script for Calibration (non-ROS)
`foo1.py` script is run after "cold start" to physically calibrate the system, when we are not sure whether the system is still static after the previous use. 
*(Calibration procedure is covered thoroughly in the Section 3.)* 

#### Node for Perception
`ttt_board_state_pub.py` node:
	- subscribes to camera stream.
	- continuously the board state making emphasis on board state change. 

#### Node for Motion
`ttt_arm_executor.py` node:
	- arm **service server**
	- makes arm motion execution centralized
	- provides to other nodes set of ROS services, like:
		- to go home.
		- to a particular storage cell.
		- particular playground cell.
#### Node for Game Control
`ttt_game_manager.py` node:
	- is **terminal driven**, running the game loop (used mainly in our project):
		- Interactive mode.
		- Autonomous mode.

## 1.4 Communication. Topics and Services

### 1.4.1 Board State
Board state **topic** `/ttt/board_matrix` is the core topic in our system. It has **type** of `std_msgs/String` with a **format** of rows connected by slash sign, like `..x/OX./...` which corresponds to the following board state: 

![[Pasted image 20260107025617.png]]

### 1.4.2 Motion Service
Motion is **service-based** (executed through services) so each call returns *success*/*failure* in an explicit way. Services:
- `/ttt/arm/go_home`
- `/ttt/arm/goto_storage`
- `/ttt/arm/goto_playground`


## 1.5 Nodes - Bigger Picture

- Below, role in system, subscription, publication, service providing, service calling for each node and one script is provided. 
- `---` means "**no**" (ex.: `ttt_arm_executor.py` node has no subscription)

![[Pasted image 20260107032410.png]]

- General flow for *terminal-driven game loop* can be pictured using **State Diagram**:

![[Pasted image 20260107041454.png]]


---

# 2. Mechanical & Hardware Design

More information regarding initial hardware configuration and later modifications can be found in Section 0: "Executive Summary & Quick Overview"

## 2.1 Platform

Platform used in our project is the tracked mobile robot Yahboom Transbot. Although the chassis provide superior off-road mobility, in the scope of our project they were more constraints, rather than advantage, since one of primary objectives in our project is precision. Initially, yaw alignment was planned to be implemented using base rotation through utilizing precision seeking techniques, like build in IMU's gyro and PID control. However, in practice surface friction variation caused constant slips and drifts. Due to that reason, the setup was modified to use build-in **pan servo** on which robotic arm was eventually mounted.

## 2.2 Robotic Arm and DOF Configuration.

After final modernization, our robotic arm gained additional Degree of Freedom in a face of pan servo. This has changed factory configurated 3-DOF arm to 4-DOF 3-axis manipulator with the following joints:
- `s9` - gripper, kept fixed since holds a magnet. Rotates in horizontal axis with respect to forearm.
- `s8` - elbow joint, rotates in vertical axis.
- `s7` - shoulder joint, rotates in vertical axis.
- `pan` - base joint, rotates in horizontal axis.
With this setup, arm can address objects located on its left and right without turning the base of the robot. This modernization enabled us to achieve acceptable level of accuracy and reduce power consumption of the system.

![[Pasted image 20260107054247.png]]
## 2.3 Arm Extension Modules

Right after mounting the robotic arm to its new position, another issue appeared - insufficient length of the arm to reach the cells. So, we have designed and 3D-printed special extensions that:
- increased shoulder link by **11 cm**.
- increased forearm link by **4.5 cm**.
With correctly utilized manipulation technique (retracting arm before each turn of pan) we could reach any cell of each of the boards, after proper manual positioning boards with respect to each other and the base of the robot with respect to the boards.

## 2.4 End-Effector

The most practical way to realize the pickup/release mechanism was to statically attach one neodymium magnet in gripper, so it can attract the figures with embedded tiny magnets during pickup operation and release the figures, snapping them off, by bringing the bottoms of the figures close enough to the cells of playground board containing sheets of cardboard with additional magnets lying under each.  
![[Pasted image 20260107054035.png]]

## 2.5 Boards and Physical Layout

- **Storage board** - 140x140mm board having thickened borders of 11.75mm with cells 31x31mm to hold the figures firmly which have 30mm of wideness. Height of board is 5mm to make the pickup easier.

![[Pasted image 20260107054051.png]]

- **Playground board** - 140x140mm board having thickened borders of 5mm with cells 40x40mm to increase the chances of releasing accurately. Height of board is 10mm to make the release easier - guide figures while they're dropping. In each cell 40x40mm cardboard sheet is laying, having magnet under each

![[Pasted image 20260107053049.png]]
## 2.6 Computing and Hardware Reliability

The onboard computer we have in our robot is Raspberry Pi 4. It is a good tradeoff of reliability and simplicity. Although continuous vision and ROS processing cause significant overloading and heating, after installing active cooling components the problem was mitigated just enough so that the system works almost flawlessly.
## 2.7 Hardware List

Overall, the hardware used in this project include:
- Yahboom Transbot.
- 4-DOF Robotic Arm with 3D-printed extensions.
- 3D-printed figures in 2 colors with embedded magnets.
- 3D-printed boards, storage and playground with cardboard sheets hiding a magnet in each cell. 
- Single end-effector magnet.
- Orbbec Astra mounted on tripod and set aside giving top down view on the playground. 
- Raspberry Pi 4 in the casing with active cooling

---

# 3. Calibration

Now after we have taken a look at our System from the general perspective it is time to dive deeper and figure out what is actually going on in each node/script. In this section we will take a look at our `foo1.py` script which itself is not a node and just a separate executable file for recalibration.  

As we have mentioned earlier, when we are not sure whether the robot is calibrated or not, or as a good practice after a long break, before we start the game we need to ensure that the system is physically calibrated. We do this in several steps. Those steps are listed below

## 3.1 Calibrating Boards

First of all we have to ensure that the **storage and playground** board are located correctly with respect to each other. We do this by:
1. Positioning their edges parallel to some object to make sure that the **boards are parallel** as well. It can be some sort object with smooth and straight surface to lean on. We kept everything simple and positioned them with respect to the *edge of the table*. By doing so we can visually see that the boards are parallel to each other, which is our goal. 
2. Setting some constant **distance between the outer edges** of the boards. In our case we use regular *ruler* that sticked between those boards, setting gap of **25mm**.
![[Pasted image 20260107085446.png]]

- To keep calibrated boards stable and rid ourselves from constant recalibration, we sticked the boards to the table using two-sided tape.

## 3.2 Calibrating Robot Base 

The next what we would like to calibrate is the base of our robot. Unfortunately or fortunately, due to its mobile nature, we cannot leash it to stay fixed in one place. Therefore it is helpful to frequently check that the base is calibrated.
Here, as well as in the case with boards, we have used the object that we found in the lab - *power inductor* (aka choke). Having a thickness of near **33mm**. we found it handy, since the distance between the robot and board allows to reach each cell without any issues. By putting this tool between the "bumper" or base and the edge of the playground board, we can correctly align the robot perpendicular to the edge of the board and roughly align the center cell of with the base of the robotic arm. 

![[Pasted image 20260107090746.png]]

However, the word "roughly" did not satisfy us, therefore we wrote some script and named it `foo1.py` to resolve this imprecision and make everything as perfect as possible.

## 3.3 Calibrating Arm Base

Below, the full script `foo1.py` used for the arm base calibration is given and explained. In a nutshell, we will move our robotic arm to some pose with its gripper open exactly to the width of the playground board's cell, and then manually slide the robot left or right to align the lips of the gripper with the side edges of the center cell in the playground board.

![[Pasted image 20260107092701.png]]

```python title:"foo1.py"
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

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

```

- So our general calibration sequence in this script is :
	- `POSE_A` -> `PAN=100` -> `POSE_B` -> `POSE_A` -> `Terminate`
- We start from `POSE_A ={"s7": 38,"s8": 230,"s9": None}` where we have set values for `s7` as *38*, `s8` as *230*; however, nothing for `s9` which with just hold its previous position  
- We want to move to the `POSE_B = {"s7": 123,"s8": 180,"s9": 107}`, where all the servos change their values, especially `s9`, since the gripper should open at width of the cell.
- `PAN_TARGET` is set to be *100*, although it should be 90, our pan servo is installed a bit askew.
- Safety first and **servo stall** is undesirable, so we also explicitly define the extremum values for each servo which matches with hardware specification:
``` python
S7_MIN,S7_MAX = 0,225
S8_MIN,S8_MAX = 30,270
S9_MIN,S9_MAX = 30,180
PAN_MIN,PAN_MAX = 0,180
```
- And according to that, we clamp every command to a particular servo to stay in safe limits:
```python
s7c = clamp(s7,S7_MIN,S7_MAX)
s8c = clamp(s8,S8_MIN,S8_MAX)
s9c = clamp(s9,S9_MIN,S9_MAX)
# ...
pan_target = clamp(PAN_TARGET,PAN_MIN,PAN_MAX)
```

- It is also important to achieve smoothness in motion. Can do that by increasing motion runtime `ARM_RUNTIME_MS = 5000`. With longer runtime we get slower movement, but less jiggle and jerk, reduced possibility of "servo conflict", reduced load on gears which can wear or even break them.
- Same applies for Pan servo for which we set:
```python
PAN_STEP_DEG = 1
PAN_STEP_DELAY_S = 0.03
```
- and therefore increment degree at which pan turns only by 1 and with 30ms time interval.

- Now to understand when to stop, it is important to know the current angles on servos `ang = bot.get_uart_servo_angle_array()`

By the those **3 steps** we can almost perfectly calibrate our system before we start the game. 

---

# 4. Perception System (Vision)

```python title:"ttt_board_state_pub.py"
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
```

## 4.1 Purpose and role in the overall system

This subsystem provides our robot with eyes – a camera. Camera takes a snapshot of the board and sends the information about as a 3x3 matrix data with any of the three possible state combinations for each cell: empty, populated with X or O. In order to make overall process autonomous we do not tell the robot when it is allowed to perform its next move. Rather, camera constantly takes snapshots of the board and compares a new value to the old one and decides accordingly.

## 4.2 ROS interface (what the node subscribes/publishes)

`ttt_board_state_pub.py` which is the main code in this module is implemented as a single node. This node has 2 communication channels:

##### `/camera/rgb/image_raw` (`sensor_msgs/Image`)

As a Subscriber it subscribes to `image_raw` topic and receives a message of type image through it. Names should be self-explanatory: our node subscribes to the topic that represents our camera to further receive an image from it.

##### `/ttt/board_matrix` (`std_msgs/String`)

As a Publisher our node publishes a state of the board as a string for `ttt_game_manager.py`, which is the game logic algorithm, to retrieve it later. All states in a message string are separated by forward slash. This method was used for ease of readability and debugging

## 4.3 High-level pipeline (image → board state)

Before sending all the necessary data to the game algorithm camera has to go through following steps:

1)    Receive an image
2)    detect the square outline of the playground board
3)    Warp it to obtain a clear top-down view on the board
4)    Extract 9 separate cells
5)    Assign a state for each cell based on HSV thresholding

All these steps are sufficient for moving on with the game logic execution. So, in the end we send a string message to the game logic code containing all the cell statuses

## 4.4 Board detection (code explanation)

Essential functions involved in board detection are `find_board_quad(gray)` and `order_points(pts)`

The goal of `find_board_quad(gray)` is to detect a board using grayscale input and this goal is achieved in next steps:

1)    `blurred = cv2.GaussianBlur(gray, (5, 5), 0)`

Apply Gaussian blur to the image to reduce noise

2)    `edges = cv2.Canny(blurred, 50, 150)`

Highlight board edges. How it works is it computes gradient and marks every spot as an edge if the gradient transition is prominent

3)    `kernel = np.ones((3, 3), np.uint8)`

`edges = cv2.dilate(edges, kernel, iterations=1)`

Dilation. At this stage the edges must’ve been already  detected. However due to external factors like shadows or lightning and internal factors like errors there may be some inconsistencies along the edges like broken pixels. Dilation thickens white pixels and bridges the gaps in edge detection

4)    `cnts = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)`

`contours = cnts[1] if len(cnts) == 3 else cnts[0]`

this part creates contours based on all detected edge regions. This turns our edges collection into a quadrilateral contour that represents our playground board

5)    This is filtering part.

```python 
image_area = w * h
if area < 0.05 * image_area:
	continue
```

Here we eliminate all the contours that occupy less than 5% of the image

```python
peri = cv2.arcLength(cnt, True)
approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
if len(approx) == 4:
...
```

Our contour region has some variations. To ensure we obtain perfect quadrilateral we reduce our contour to a polygon with 4 vertices. 0.02 * perimeter is called tolerance. This is tunable and based on constant decides how many points there will be. If vertices number approximation is 4 we proceed

```python
x, y, w_box, h_box = cv2.boundingRect(pts)
aspect = float(w_box) / float(h_box) if h_box > 0 else
if 0.7 < aspect < 1.3:
    return pts
```

This part checks if our polygon is a square like our playground board by calculating ratio width/height. 0.7-1.3 are acceptable boundaries for the ratio for our shape to be considered a square

## 4.5 Picture Warp

We have obtained an almost perfect square for our playground board in the previous step. However, it is only square relative to the ground, not to the camera. To divide our playground in its 9 cells we need a playground picture to appear perfectly square, you may call it from “bird eyes’ view”. We call this transition process warping

First, before warping we need to order our points using `order_points(quad)`. quad is the square polygon we obtained from edge detection. Here is how `order_points` works:

```python
def order_points(pts):
    rect = np.zeros((4, 2), dtype="float32")
    
    s = pts.sum(axis=1)
    rect[0] = pts[np.argmin(s)]  # top-left
    rect[2] = pts[np.argmax(s)]  # bottom-right
    
    diff = np.diff(pts, axis=1)
    rect[1] = pts[np.argmin(diff)]  # top-right
    rect[3] = pts[np.argmax(diff)]  # bottom-left
    
    return rect
```

To order our points we specify the position for each corner. We First we assume x increases towards right of the image and y increases towards the bottom. This implies that in the top left corner coordinate sum is minimum and in the bottom right corner its maximum. We map 0 vertex of the square to top-left and 2 for bottom-right. Then we apply similar procedure for difference between y and x. Where the difference is minimum it is going to be top-right corner and where it is maximum – bottom-left corner. Then as map corresponding vertices to the values.

```python
WARP_SIZE = 300

dst = np.array([
    [0, 0],
    [WARP_SIZE - 1, 0],
    [WARP_SIZE - 1, WARP_SIZE - 1],
    [0, WARP_SIZE - 1]
], dtype="float32")
```

Next, we specify warp dimensions. It is going to be 300x300 pixels. It is good because our playground is square, too

`M = cv2.getPerspectiveTransform(pts, dst)`

Here we take our updated square and do the so-called homography transformation. This function takes all points in the square and maps them onto our warping frame

`warped = cv2.warpPerspective(frame, M, (WARP_SIZE, WARP_SIZE))`

In the end we, finally,  warp our transformed image. It is important to not that our frame is BGR, not the grayscale. We transitioned back to BGR because we will need It to distinguish cells based on their state using HSV thresholding.

## 4.6 Cell extraction

After warping our image we need to divide it into 9 cells. The problem with just cutting the image into 9 similar fragments is the errors. When determining the cell status we only take into account the brightness of the cell (V in HSV). However, with naive division we will encounter border color and most probably border shadow that will produce unpleasant errors on thresholding stage

`cells_3x3 = extract_cells(warped)   # 9 raw cell crops (include borders)`

as the code and a comment next to it suggest we extract cells from the image without bothering about playground borders for now. What extract_cells does:

```python
H, W = warped.shape[:2]
ch = H // 3
cw = W // 3 
``` 
  
since our grid and image have stable coordinate systems, to find h and w for each cell we just divide image height and with by 3

`cell = warped[r*ch:(r+1)*ch, c*cw:(c+1)*cw]`

Next we set the ranges in pixels based on row/column and calculated height and height and width for each cell.

After initial cropping we can start addressing error minimization (borders and shadows).

```python
CELL_CROP_MODE = "frac"
CELL_MARGIN_FRAC = 0.50
```

This bit means that for cropping we will use fraction of the extracted cells

```python
h, w = cell.shape[:2]
mh = int(h * CELL_MARGIN_FRAC / 2.0)
mw = int(w * CELL_MARGIN_FRAC / 2.0)
inner = cell[mh:h-mh, mw:w-mw]
```

Since we calculated that each cell is initially 100px, with the specified value for fraction, we remove 25px from both ends of height and width

## 4.7 HSV thresholding

After we extracted and compressed our 9 cells, its finally time to establish thresholds to distinguish between cell statuses: X, O, Empty.

`v = avg_v_bgr(cell_bgr)` computes brightness (V) Based on Average BGR  color in the cell using `hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)`

`sym = classify_by_v(v)` simply returns the cell status based on the V and thresholds set for each color:  
``` python
V_X_MAX     = 130
V_EMPTY_MIN = 140
V_EMPTY_MAX = 200
V_O_MIN     = 220
```
Threshold were carefully picked after running different grid configurations in different lightning envirronments.

`compute_matrix_from_cells(cells_3x3)` inherits `avg_v_bgr` functionality. All it does is it computes average V for all cells at once

That’s all for vision. What issues we encountered with it and what lessons we learned will all be discussed in the lessons and limitations section of the report.

---

# 5. Motion & Manipulation

In this section we will talk about `ttt_arm_executor.py` which itself is a manipulation server that converts requests such as "go to (row, col)" into real physical motions realizing the contact with physical environment and letting the robot to interact with it. 

Let's take a look at a code and then discuss it:

```python title:"ttt_arm_executor.py"
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
```

## 5.1 Main Purpose

This subsystem which is responsible for the execution of repeatable physical moves required by the game logic does the following:
1. reaches particular cell of either playground or storage
2. returns to home state after each action
3. ensures that the movement is smooth without "servo conflict"

From the architectural perspective the `ttt_arm_executor.py` behaves more like motion server as it exposes a set of services:
- `/ttt/arm/go_home`
- `/ttt/arm/goto_storage`
- `/ttt/arm/goto_playground`

This kind of job division is crucial as it is increasing the reliability so that the game logic never directly influences servo angles, timings, smoothing => it just calls services.

## 5.2 Final Motion Concept

- `PAN` (yaw servo) - used for horizontal move.
- `s8` + `s7` - operate in vertical plane.
- `s9` - kept constant (unless we do calibration).

## 5.3 Substitute for Inverse Kinematics

Since one of the biggest assumptions of our project states that the system should be mostly static (except robotic arm) using inverse kinematics is not rational. Due to home-made robotic arm configuration, the `s7` servo is offset from the base center and elevated, making the measure of rotation center complicated and susceptible to imprecise value. Additionally the correct coordinate for each cell is hard to be calculated. Therefore, we have decided, instead of Inverse Kinematics, to find servo angles corresponding to each cell in each board we will use basic **data logging**.

By disabling tension on servos, we manually moved the end-effector to the center cell and logged the values for servos, so that, suppose to reach cell (1, 1) in playground, the values are (124,174,170,100) for s7, s8, s9, and PAN respectively. Then using math and some estimations we figured out the values for the remaining cells and got Poses to be:

```python
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
```
 
## 5.4 Home Pose 

As we have discussed before, the ensure stability, repeatability and avoid any damages to the system, after each move we should return to the home pose which is defined to be for `servos` and `PAN` as:

```python
HOME = {"s7":38,"s8":230,"s9":170}
HOME_PAN = 90
```

## 5.5 The Service Interface

Referring to the previously stated idea, in the following subsystem we have 3 ROS services:
- `/ttt/arm/go_home` - **Trigger**, to return the arm to home position.  In returns `success=True/False`, status message.
- `/ttt/arm/goto_storage` - **GotoCell**, to move to a desired cell given (row, col). It responses with `ok=True/False`, message `OK: storage (r,c)` or just an error string.
- `/ttt/arm/goto_playground` - **GotoCell**, same concept, however for the playground board.

Exactly this, is the interface expected by `ttt_game_manager.py`.

So, in summary, `ttt_arm_executor.py` does not has the decision mechanism, it just a tool to execute the motion commanded by `ttt_game_manager.py`.

## 5.6 Safety Limits

Just as in `foo1.py` in `ttt_arm_executor.py` we should define thresholds for servo and PAN motions to clamp the input value before it makes affect to avoid the risks of stalls and abrupt snaps. So we set those values:

```python
S7: 0..225
S8: 30..270
S9: 30..180
PAN: 0..180
```

## 5.7 Key Idea - Ordered Actuation

Based on our experience of running multiple codes to achieve smooth manipulation, we faced many issues with unpredictable arm behavior: sudden jiggles, no response, accelerated start and so on. We managed to minimize those by reducing the speed and setting more distinct thresholds to the amplitude of each servo, however there still were chances for this problems to occur. So to make sure that everything is completely safe and 100% operational, we decided to set the orders at which each servo, including pan, will actuate.

In the scope of our game logic we can distinct two types of arm motion pattern each has the order of servo actuation:
- HOME -> CELL (`home_to_cell`):
	1. **PAN** goes first, we rotate to face the target cell while the arm is still folded.
	2. **s8** goes next, while s7 stays stable to avoid "digging" into the table.
	3. **s7** finalizes the move of grabbing, laying gripper's tip exactly in the center of the cell.
		
- CELL -> HOME (`cell_to_home`):
	1. **s7**
	2. **s8**
	3. **PAN**

## 5.8 Smooth pan

- PAN is moved using `smooth_pan()` in small increments as in **foo1.py**:
	- `PAN_STEP_DEG = 1`
	- `PAN_STEP_DELAY_S = 0.02`

## 5.9 "Atomic" Motion Primitive

`arm_cmd` - lowest level command that is used for servos s7, s8, s9 to control the speed and reduce servo conflict to increase the stability. Its work can be observed more clearly when the arm reaches for the farthest cell - the speed of servos are increased. So as a result, we get more or less similar time for grabbing/placing all figures. It:
1. clamp each of the existing joints
2. send single UART type command:
	`bot.set_uart_servo_angle_array(s7, s8, s9, runtime_ms)`
3. wait until it completes:
	`time.sleep(runtime_ms/1000 + ARM_SETTLE_S)`

## 5.10 Storing a "state" Dictionary

`self.state = {"s7": ..., "s8": ..., "s9": ..., "pan": ...}`

- is not a sensor feedback but the last commanded angle which is used for the following reason:
	  - during `cell_to_home()`, St1 needs `s8_hold = self.state["s8"]` as a result `s7` can return home at the same time while `s8` stays fixed.
- Also the way to mitigate unintended behavior in the middle of the game to achieve smoothness and flawlessness. 

## 5.11 Execution Logic

```python
self.cell_to_home()
self.home_to_cell(POSES[board][(row, col)])
```

So the overall service execution logic can be observed like this:
	- all the motion requests go through `_srv_goto()`:
		1. validates (row, col) exists in the pose table, meaning that sent request is correct.
		2. return to home firstly
		3. execute the home -> cell sequencing
		4. finally, returns success response

## 5.12 What is out of Scope of This Node

- `ttt_arm_executor.py` is not responsible:
	- for perception or checking the state of the board.
	- game decision

- Only responsible for reaching the cells. From code perspective very similar to `foo1.py` however expands to services and is a ROS node.

---

# 6. Game Logic & Decision-Making

Finally, we have reached the core functionality of our system - the decision making algorithm through minimax in interactive mode of play and random in autonomous realized in `ttt_game_manager.py` which itself is a teminal-driven and provides an additional functionality of choosing an Active (starts with X) and a Passive (plays with O) player in interactive mode. 

Shortly, this layer of our System is responsible for **consuming** the *board state* from vision and **commanding** *arm services*.

```python title:"ttt_game_manager.py"
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
```
## 6.1 Inputs and Outputs

- **Inputs** coming from the vision are board state information which are necessary to stay constantly updated for any changes appeared:
	- Topic is `/ttt/board_matrix` with `std_msgs/String` message type. 
	- Have format of 3 rows separated by "/" (ex.: `"O.X/XO./... `)
	- Updated when the vision node publishes: periodic - every 5 seconds, and on change.

- **Inputs** for user mode selection provide an additional functionality of choosing an Active (starts with X) and a Passive (plays with O) player in interactive mode.

- **Outputs**:
	- Arm Commands (discussed before)
		- `/ttt/arm/goto_storage`
		- `/ttt/arm/playground`
		- `/ttt/arm/go_home`
	- Debug:
		- *logs* along with board state in the form of *matrix* being constantly printed in terminal.

## 6.2 Board Representation made Internally 

#### Board parsing
Using `parse_board_string()` we convert given string into 3by3 matrix of characters and get `'X'` , `'O'`, `'.'`
If the parsing has been made incorrectly and fails, the whole message is being ignored 

#### Win Detection
We have preset win combinations that correspond to 3 rows, 3 columns, 2 diagonals, overall - 8 lines:
```python
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
```

so the winner(board) scans through these lines and returns  `'X'` , `'O'`, `None`.

#### Termination Conditions
We stop the game in two cases:
1. when robot is won (it is impossible to win for human) `winner != None`
2. board is full `is_full(board)` 

## 6.3 Turn Management A/P Player support

Important to mention that the game logic in our case emphasize that player with X figure chosen moves first. So that, user selects to play as an Active player (play first and with X) or as a Passive player (with O).

Due to that reason we have the variable `turn` that is always initialized as "X". After that the logic checks if that "X" refers to `HUMAN` or `ROBOT`

With this design we avoid the duplication of code.

## 6.4 Interactive Mode Logic

Interactive logics designed around the fact that the robot should observe the change in the board state to make sure that the human being has indeed made its move so the robot can move next. This verification process is crucial, since without it, the game flow will turn into a total mess.

Key loop variables used in this design are: `last_seen = latest_board_str` which stores the last published board string; `latest_board_str ! = last_seen` means that the human move has been detected (aka change has been detected).

#### Human Turn
when it is human's turn the system displays "Waiting for human move..." and waits till the vision publishes different board string, then, prints updated board.

#### Robot Turn (using minimax)
step 1 -> taking a snapshot of board 
step 2 -> running minimax to get `(score, best_move)`
step 3 -> executing the placement using `place_piece(robot_symbol, r, c)`
step 4-> waiting for vision confirmation that the board changed
step 5 -> printing the newly updated board

Therefore, the robot's move is thought to be complete only after both are true:
	- arm services have been executed successfully
	- perception has confirmed that the change was expected 

## 6.5 Minimax

Minimax algorithm in our project represents a full-depth search and since the tic-tac-toe game itself is very simple, the minimax solves it quickly.

#### Evaluation in Minimax
Shortly,
`+1` if robot wins
`-1` if human player wins
`0` if result is draw
`none` if our game has not finished yet

#### Recursion Strategy
If `maximizing =True` then robot tries to maximize the returned value; otherwise - opponent minimizes it.
For each move: we place symbol temporarily -> recurse -> undo the move.

Tic tac toe having only 9-cells to move in has very small branching factor if we consider it with other games, like checkers or chess, searching until the terminal states happens quickly and it is very practical, giving straightforward pattern:
	- if a **win exist**, minimax definitely **finds** it.
	- if **no win**, minimax tends to **force a draw**.

## 6.6 Autonomous Mode Logic

In autonomous mode we are using the same control structure:
	- check board -> decide -> make move -> verify -> repeat

However, the biggest difference is that instead of minimax, decision is made randomly using `random.choice(legal_moves(board))`

## 6.7 Storage Pieces

Internal piece availability list is:
```python
STORAGE_X = [(0,0),(0,1),(0,2),(1,0),(1,1)]   
STORAGE_O = [(1,2),(2,0),(2,1),(2,2)]
```

While playing:
	- placing X will pop the first coordinate from `available_x`
	- placing O will pop from `available_0`

## 6.8 Failure Handling & Timeouts

We have common failure modes that were considered in our program:
1. **No board received** (detected)
	- `wait_for_board(timeout=20s)`, if this fails the game will be exited with error. This type of fail refers to camera and vision startup issues
2. **Arm service failure**
	- when either `/goto_storage` or `/goto_playground` returns `ok=False` then the game will be immediately stopped.
	- if `/go_home` fails -> stop.
	- this is helpful to avoid the case when we may continue with an unknown physical state.
3. **No vision Confirm after robot move**
	- robot will wait until the board state changes. If stuck, can reset game with Ctrl+C. 

---

# 7. Discussion and Conclusion

## Discussion

Let’s now recap what we have done by discussing limitations we encountered on our way to completion of the project.

### Early encounters.
On our first steps, project greets us with Nvidia Jetson Nano. Normally, we would install Raspberry Pi beforehand since:

1)    Yahboom Transbot offers such an option
2)    We had access to it
3)    We already had a little experience working on Raspberry Pi on PiCar robot

However, Jetson came preassembled, so we though we could give it a shot. At first, everything worked just fine. Unfortunately, not much time passed before Jetson wronged us for the first time. While rebooting the robot, we found out that all system files were corrupted. At the time we didn’t think Jetson was the issue, so we had no other choice but to reinstall the whole operating system. After this happened again, we figured we might want to change hardware. As soon as we transitioned to Raspberry Pi, we never had to troubleshoot system errors again.

### Tracks
Hands down, the part of the project that made us anxious the most. Initially, we wanted to use tracks to mimic pan movement of classical 3-DOF (Degrees of Freedom) robotic arm. The idea was to rotate the robot around its axis to align it with boards and cells. Unfortunately, no matter how many control methods we tried to utilize and how many parameters we tuned, performance was never almost perfect. Due to physical characteristics of the tracks and how they interacted with the environment one track was always ahead of the other. This caused robot to drift, especially, with more iterations. A very spontaneous decision to switch to pan servo was made during the last weeks of the semester.

### Arm placement
When we switched arm position from the front of the robot, we knew it would not have enough length to reach the board. Going back to track was not an option. We had no other choice but decide how we could make the arm longer. The only and most effective option was to design 3-d printable pieces. We calculate potential length of the arm relative to each other and the board placement. Then we build a design and print it.

### Vision
At first, we wanted to go for depth detection for the board and pieces. When we turned depth mode of the camera it turned out the pieces were too small to be properly detected. Then we decided to switch to HSV detection – specifically V, which is brightness and was discussed in details in section of the report with corresponding name.

### Hardware overheat.
As our project grew, we noticed that our Raspberry Pi CPU became progressively hotter. At first, we wanted to mount a radiator but because we didn’t have quick access for one, we had to look for another option. We approached Mr. Vahabli and he turned out to have a special case with a fan for model of Raspberry Pi we used.

## Conclusion

In this project Yahboom Transbot was made to play a popular game of Tic-tac-toe either against a human or itself. Easy at first glance, this project combines a lot of different fields very important for robotics:

1)    We learned to use Robot Operating System. Concepts like nodes, topics, services, messages are all crucial to projects like this and help to establish communication network between the processes.
2)    With Astra Orbbec we dive deeper into the world of computer vision. Edge detection, refactoring an image, HSV data are all concepts important in computer vision and we answered the question of how to use them using OpenCV library.
3)    Solving robotic arm actuation gave us an insight to how we can work with bodies with 3-DOF and how the arm can communicate with an environment.
4)    Additionally, what does not directly involve robotics, we learned and implemented a MiniMax algorithm, and us and our robot are now unbeatable in TicTacToe.

We built, destroyed and rebuilt. What first appeared as a just a silly fun project became a series of hardships to endure. But in the end, we did it and it made us stronger. We hope this project will spark a flame of enthusiasm for future generations and help them with building a project of their own and use our experience for guidance.

---
# 8. References

[1] ROS, “std_srvs/Trigger,” ROS Wiki. [Online]. Available: http://wiki.ros.org/std_srvs/Trigger.
[2] Open Source Robotics Foundation (OSRF), “Catkin configuration overview (ROS Melodic),” docs.ros.org. [Online]. Available: https://wiki.ros.org/catkin/conceptual_overview.
[3] Yahboom Technology, “Yahboom Transbot (Raspberry Pi 4B) documentation / study page,” Yahboom. [Online]. Available: https://www.yahboom.net/study/ROS-Transbot-RASPI4B.
[4] YahboomTechnology, “Trasnbot_Pi (Transbot Pi) repository,” GitHub. [Online]. Available: https://github.com/YahboomTechnology/Trasnbot_Pi.
[5] ORBBEC, “ros_astra_camera (Astra camera ROS driver) repository,” GitHub. [Online]. Available: https://github.com/orbbec/ros_astra_camera.
[6] ORBBEC, “Orbbec SDK,” ORBBEC Developers. [Online]. Available: https://www.orbbec.com/developers/orbbec-sdk/.
[7] ORBBEC, “Astra Mini S Datasheet,” Reichelt (PDF). [Online]. Available: https://cdn-reichelt.de/documents/datenblatt/C300/ORBBEC_ASTRA_MINI_S.pdf.
[8] OpenCV, “Contours: Getting Started,” OpenCV Documentation. [Online]. Available: https://docs.opencv.org/4.x/d4/d73/tutorial_py_contours_begin.html.
[9] OpenCV, “Geometric Image Transformations (Perspective transform / warpPerspective),” OpenCV Documentation. [Online]. Available: https://docs.opencv.org/4.x/da/d54/group__imgproc__transform.html.

---

# 9. GitHub and YouTube 
