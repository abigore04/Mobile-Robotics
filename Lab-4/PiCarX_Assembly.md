# Assembly and Getting Started with SunFounder PiCar-X robot

**Course:** Mobile Robotics  
**Student:** Farid Ibadov.  **Team-mate:** Said Ahadov.  **Instructor:** Alexzander Leonard Farral.  **University:** ADA University  
**Date:** October 2025  
**Platform:** EV3Dev (Debian-based OS for LEGO EV3)  
**Language:** Python (EV3DEV2 library)

## Outline

- [Objectives](#objectives)
- [Hardware Overview](#hardware-overview)
- [Assembly](#assembly)
- [Installing appropriate images and Packages](#installing-appropriate-images-and-packages)


## Objectives

The scope of this lab report covers the discussion of hardware components, assembly of PICAR-X robot, and initial configuration of Raspberry-Pi 4.

## Hardware Overview

<img width="473" height="671" alt="Pasted image 20251102193742" src="https://github.com/user-attachments/assets/acc1ffda-e553-4066-bd0c-12e7f78ac89c" />

When we first open the box, we can see there the following details:
- structural plate, consisting of the main frame, steering levers, camera casing and etc.
	- made out of aluminum with factory-made holes for attachment and cable management.
- 2 pairs of front and rear wheels.
	- plastic wheels with rubber tires 
	- rear wheels are connected and driven by TT DC Motors and can rotate clock- and counterclockwise.
	- front wheels spin freely and are connected to the steering levers, that are moved by steering servo.
- Battery
	- Lithium-ion 2-cell rechargeable battery with 2000mAh capacity that can power robot HAT with all motors, servos, sensors, Raspberry Pi. 
- FPC Cable
	- Flat Printed Circuit cable which connects camera module to the Raspberry Pi's CSI (Camera Serial Interface) port.
- a pair of FFC Cables
	- Flexible Flat Cables, not used.
- a pair of 5-pin wire
	- for grayscale module
- a pair of 4-pin wire
	- for ultrasonic
- USB-C cable
	- for charging
- ultrasonic module
	- to measure proximity using echo pulses
- camera module
	- camera with normal resolution
- grayscale module
	- like color sensor but consist of a 3 rows of infrared sensors detecting values between white and black (gray tones)
- a pair of TT DC motors 
	- DC motors having the ability to spin in 2 direction, but do not have turn sensors.
- 3 servo motors
	- 1 for pan and 1 for tilt of camera, 1 for steering
- robot HAT ([1])
	- Motor port left/right – 2-channel XH2.54 ports, one for the left motors connected to GPIO 4 and the other for the right motors connected to GPIO 5.
	- 2x I2C pin from Raspberry Pi
	- PWM – 12x PWM (P0-P11)
	- 4x ADC pins (A0-A3)
	- 4x digital pins (D0-D3)
	- Battery status indicators
	    -  LED 2 lights up when the voltage is more than 7.8 volts
	    -  LED 1 lights up when the voltage is between 6.7 – 7.8 volts
	    - Both LEDs turn off when the voltage is lower than 6.7 volts
	- Supply Voltage – 7 – 12V DC via a 2-pin PH2.0 connector that can be used to power the Raspberry Pi at the same time
- screws, washers, rivets, nuts, standoffs, electrical tape, Velcro sticker, tools

*Note: that Raspberry Pi is not included in a kit.*

## Assembly

1. Secure four M2.5x18+6 standoffs on the largest structural plate (A) from the flat side using four M2.5x6 screws.
2. Mount the Raspberry Pi on top of those standoffs and secure it with M2.5x11 standoffs (**important**: make sure the model of the Raspberry Pi is appropriate for particular standoffs, due to some ports, as an RF-45 Ethernet port, the height can be greater which can hinder the further mounting of the Robot HAT, if that a problem - try other standoffs).

    <img width="960" height="1280" alt="Pasted image 20251102210310" src="https://github.com/user-attachments/assets/84a7ebc5-28d2-437f-ba67-011c296df406" />

3. Lift the black lever on camera connection port on Raspberry Pi and insert the FFC cable (make sure you insert FFC with the blue side towards the black lever, otherwise you can damage the cable while closing the lever) and close the lever.
4. Mount Robot HAT on top of the Raspberry Pi and secure with M2.5x6 screws.
5. Turn the structural plate over and place both motors on their designated places by securing them using a pair of M3x25 screws, spring washers, and M3 nuts for each motor. Make sure that metal heads of the motors face to the back and cables face inward. Then pull the wires of motors through the hole in the rear part of structural plate and label each cable pair with stickers with descriptive names on them (like, ML, MR). It is recommended to label cables to make the process of plugging and wiring easier.  

    <img width="960" height="1280" alt="Pasted image 20251102201736" src="https://github.com/user-attachments/assets/8095ef1c-55df-4e11-9d3e-29e858c1d1c6" />

    <img width="1280" height="960" alt="Pasted image 20251102204705" src="https://github.com/user-attachments/assets/c3e5e0c3-1f52-42c3-8e8e-cb9107ea4122" />

6. Apply one part of Velcro sticker to the middle-bottom of the structural plate and another to the battery itself. Then, attach the battery, but before that, make sure that wires head the rear side of the plate and then pull the wires though the rectangular hole.
7. Attach battery wires to the specific port on Robot HAT for power supply. Orientation matters (due to polarity), if the clip does not go in, flip it and try again. Pay attention to the "horns" - they should face upwards.

    <img width="960" height="1280" alt="Pasted image 20251102202135" src="https://github.com/user-attachments/assets/676bd886-f899-4a5f-9845-4fe77b2cb969" />

8. Secure dual-sided servo arm for pan servo using 4 M1.5x3 screws at the front of the structural plate
9. Insert ultrasonic sensor to the front - just insert it "eyes" through two big holes making sure that the port for cable faces up. Then sandwich the sensor with another plate of the same size (H plate) and secure this all with 2 R3090 rivets.
10. Using a pair M3x6 screws from the top, attach a pair of M3x26 standoffs from the bottom - near ultrasonic sensor.
11. Remove the sticker from the camera's back to attach it securely to its own board, then insert FFC cable and close the lever. Then, secure camera inside its aluminum casing (C plate) using 4 R2048 rivets. Then place a dual-sided servo arm for the outside of camera casing and secure with M1.5x3 screws from the inside.
12. Insert one of servos into B plate. Imagine a plate being a chair, place it so its seat faces you. Then mount servo to the "seat's" hole, making sure that the cables face to the right. Secure it with 2 R2056 rivets.  
13. Install another servo into the "back of a chair", so here as well, cables head to the right. Secure it with 2 R2056 rivets. (double check that both servos' shafts are on the same side).
14. After that, insert this structure inside camera casing (C plate) and secure the structure using finest screw form servo package. Then mount the final structure to the designated place on the front of structural plate and secure that from the bottom using finest screw. By the end of gimble assembly, it should be something like this:

    <img width="960" height="1280" alt="Pasted image 20251102204137" src="https://github.com/user-attachments/assets/b48e4a38-2360-47bb-bd2f-97819f0cf476" />

15. Attach stickers with labels on each servo cable. Bottom one is for Pan, upper is for Tilt. 
16. Attach remaining servo for steering using a pair of R2056 Rivets, label the cable. Cable should face to the left, like this:

    <img width="1752" height="1374" alt="Pasted image 20251102205015" src="https://github.com/user-attachments/assets/d614553a-9788-4204-99c6-7f8bf37ae93f" />

	
17. Attach servo arm (single sided) to the G plate using M1.5x3 screw (don't tighten up - let it rotate a bit). Rotate structural plate (A) and attach this to the servo shaft and secure with fine screw.
18. Secure big base plate (D plate) at the bottom using M3x6 screws.
19. Attach grayscale to the top from the bottom of base plate using R3055 rivets, so port of grayscale sensor faces bottom back.
20. Plug the 4-pin wire to the ultrasonic module (pay attention to clips - attach properly) and label it. Pass the wire through the hole near steering servo. We will plug another end to the appropriate Digital Pins on Robot HAT later.
21. Plug the 5-pin wire to the grayscale module (pay attention to clips - attach properly) and label it. Pass the wire through the same hole near steering servo. We will plug another end to the appropriate ADC Pins on Robot HAT later.

    <img width="960" height="1280" alt="Pasted image 20251102210348" src="https://github.com/user-attachments/assets/da5b94ce-013d-4dce-9dc2-1746a150efda" />

22. Then, using 3 R3065 rivets from each side attach F and E plates to mount front wheels on them later using 1 R30185 rivet and 3 washers between rim and plate on each side. 
23. Attach rear wheels.
24. Turn on Robot HAT using on/off switch. Then one by one, attach servos' cables to P11 pins and press zero button to reset them. (later we can reset the position of servos either in Terminal or in GUI of "ezblock")

    <img width="1841" height="2560" alt="Pasted image 20251102211215" src="https://github.com/user-attachments/assets/a769289e-6738-4503-8a59-865ef3d4b671" />
	
25. Finally having all cables attached to modules, you can refer to the labels and attach them to Robot HAT pins by using this instructions. So, basically, servos go to PWM (since some motors and servos work on a Pulse Width Modulation principle, when the motor is ON for a particular period of time), grayscale to ADC (since analog signals are needed to be converted to digital), ultrasonic to Digital (since it already sends digital signals).

    <img width="369" height="237" alt="Pasted image 20251102211522" src="https://github.com/user-attachments/assets/e1b770e1-935c-43d5-9900-1f66ea5ec622" />

26. You should get something like this:

    <img width="277" height="224" alt="Pasted image 20251102211618" src="https://github.com/user-attachments/assets/77d7c199-e7ff-4804-901b-3a6a4b1160e9" />

27. Finally, you can make cable management and wrap all cables, coming from the front, with cable wrap (however it is recommended to do this after testing if the hardware works properly).

## Installing appropriate images and Packages

After completing the assembly, images should be installed on micro SD card which will be plugged in a dedicated socket on Raspberry Pi:
1. Download official Raspberry Pi imager by following this
  [link](https://www.raspberrypi.com/software/)
2. Plug micro SD to your PC and format it.
3. When you run imager, you should choose your SD card
4. Then choose your Raspberry Pi model in Operating System tab (4 in our case)
5. Then you should set login and password to it and can initially specify some network, although, it can be done later.
6. Then press write and wait, it will take a while.
7. After finishing up, plug micro SD cart into Raspberry Pi's  socket:
    <img width="1200" height="857" alt="Pasted image 20251102213506" src="https://github.com/user-attachments/assets/8b2173ad-c93b-49f0-b82f-a42aecdadca9" />
8. Turn on Robot HAT or plug in USB-C cable to Robot HAT's port if battery is low and let the Raspberry Pi to boot.
9. After that, Connect your mouse and keyboard to USB ports and external monitor with HDMI using HDMI-to-micro-HDMI adapter.
10. Once it boots up, you will see Linux GUI, just enter login and password specified before, and that's it, the only thing left is to install some packages for PiCar-X specifically, but I had experienced issues due to problems with network access on campus (I have tried my best to achieve stable connection, but it interrupted continuously; I have tried share mobile hotspot, pc hotspot, create a network bridge, share using USB - all failed; on-campus firewall system blocks access on unauthorized devices. At some point I managed to install some libraries and have tested the servos and motor, and they worked (can check on my picar) but I forgot to record that).

## References

[1] J.-L. Aufranc (CNXSoft), “SunFounder PiCar-X 2.0 review – A Raspberry Pi 4 AI robot car programmable with Blockly or Python,” _CNX Software_, Jan. 11, 2024. [Online]. Available: [https://www.cnx-software.com/2024/01/11/sunfounder-picar-x-2-0-review-raspberry-pi-4-ai-robot-car-blockly-python/](https://www.cnx-software.com/2024/01/11/sunfounder-picar-x-2-0-review-raspberry-pi-4-ai-robot-car-blockly-python/)

[2] SunFounder, “_SunFounder PiCar-X 2.0 Kit for Raspberry Pi – Assembly Tutorial_,” YouTube, Video, Jan. 12, 2024. [Online]. Available: [https://www.youtube.com/watch?v=i5FpY3FAcyA&t=1s](https://www.youtube.com/watch?v=i5FpY3FAcyA&t=1s)
