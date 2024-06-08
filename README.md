# 4 DoF Robotic Arm
3D-printed robotic arm using Raspberry Pi, ROS, and Python. Based on Open Sourced [LiteArm i2](https://www.thingiverse.com/thing:480446) Design.

<p align="center">
 <img src="https://github.com/estods3/raspberrypi-LiteArmi2/blob/master/rosreplay.gif" alt="video" width="600"/>
</p>

## Hardware

### Frame
Based on Open Sourced [LiteArm i2](https://www.thingiverse.com/thing:480446) Design.

### Electronics
Controller circuit to power/control the arm was built as a RaspberryPi Shield/Hat to consolidate hardware.

<p align="center">
  <img src="https://github.com/estods3/raspberrypi-LiteArmi2/blob/master/robotarm_circuitboard.jpg" width="600"/>
<p align="center">

#### Bill of Materials
The table below contains the parts used to create the controller circuit. Additionally, red (5V), green (ground), and yellow (signals) solid core wire was used to make the connections.

<div align="center">

| Part | Quantity | Link |
|:----:|:----:|:----:|
| Raspberry Pi | 1 | |
| Servos | 3 | |
| Perfboard (14x20) | 1 | |
| 11x1 Pin Male Headers | 1 |
| 20x2 Pin Female Headers | 1 | |
| DC barrel jack | 1 | |
| Electrolytic Capacitor | 1 | |
| Capacitor | 1 | |
| Power Cable | 1 | |
| Ethernet Cable | 1 | |
 </div>

#### Circuit Diagram
The controller circuit serves two main functions: 1) Power the motors and RaspberryPi off of a single 5V DC power supply (DC barrel jack + decoupling capacitors) and 2) breakout signals, power, and ground to header pins for the motors.

<div align="center">
11x1-Pin Male Headers Pinout
 
| 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 | 11 |
|:----:|:----:|:----:|:----:|:----:|:----:|:----:|:----:|:----:|:----:|:----:|
| NC | NC | Ground | 5V | Signal-Wrist | Ground | 5V | Signal-Extension | Ground | 5V | Signal-Base |
</div>
These headers are used to easily connect servos to board for power and control signals. Connect the servos as follows:

* Servo Wrist: Connect to pins 3-5
* Servo Extension: Connect to pins 6-8
* Servo Base: Connect to pins 9-11

NOTE: Pins 1 and 2 are "Not Connected" and were added for future use to power/control an end effector such as an electromagnet. 

<div align="center">
20x2-Pin Female Headers 
 
| Column 1 | Column 2 |
|:----:|:----:|
| | 5V | 
| | 5V |
| | |
| | |
| | Signal-Base (GPIO18) |
| | |
| | |
| | |
| | |
| | |
| | |
| Ground | |
| | |
| | |
| | |
| | Signal-Extension (GPIO13) |
| Signal-Wrist (GPIO12) | |
| | |
| | |
| Ground | |
</div>

The 20x2-pin female headers were placed on the underside of the perfoard to connect directly to the 20x2 set of male Raspberry Pi GPIO Pins.

NOTE: Position decoupling capacitors as close to 5V GPIO pins as possible.


## Software

#### GPIO Pin Assignment
The following pin assignment was used for each servo (ensure the appropriate servo is connected to the right location on the 11x1 Male Header Connector.

<div align="center">
| Servo | GPIO Pin |
|:----:|:----:|
| Base | GPIO18 |
| Extension | GPIO13 |
| Wrist | GPIO12 |
</div>


### ROS

### Kinematics
https://github.com/AliShug/EvoArm

### Mode 1: Remote Control and Playback
