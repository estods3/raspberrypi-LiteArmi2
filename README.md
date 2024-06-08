# 4 DoF Robotic Arm
3D-printed robotic arm using Raspberry Pi, ROS, and Python. Based on Open Sourced [LiteArm i2](https://www.thingiverse.com/thing:480446) Design.

<p align="center">
 <img src="https://github.com/estods3/raspberrypi-LiteArmi2/blob/master/rosreplay.gif" alt="video" width="600"/>
</p>

## Hardware

### Frame
Based on Open Sourced [LiteArm i2](https://www.thingiverse.com/thing:480446) Design.

### Electronics
Circuit to power/control the arm was built as a RaspberryPi Shield/Hat to consolidate hardware.

<p align="center">
  <img src="https://github.com/estods3/raspberrypi-LiteArmi2/blob/master/robotarm_circuitboard.jpg" width="600"/>
<p align="center">

#### Bill of Materials
The table below contains the parts used to create the circuit. Additionally, red (5V), green (ground), and yellow (signals) solid core wire was used.

| Part | Quantity | Link |
|:----:|:----:|:----:|
| Raspberry Pi | 1 | |
| Servos | 3 | |
| Perfboard | 1 | |
| 10 Pin Male Headers | 1 |
| DC barrel jack | 1 | |
| Electrolytic Capacitor | 1 | |
| Capacitor | 1 | |
| Power Cable | 1 | |
| Ethernet Cable | 1 | |
 
#### Circuit Diagram
The circuit serves two main functions: 1) Power the motors and RaspberryPi off of a single 5V DC power supply (DC barrel jack + decoupling capacitors) and 2) breakout signals, power, and ground to header pins for the motors.
 
 ________________
|                |
|                |
|                |
|                |
|                |
|                |
|                |
|                |
|________________|

NOTE: Position decoupling capacitors as close as possible to 5V GPIO pins as possible

## Software

### ROS

### Kinematics
https://github.com/AliShug/EvoArm

### Mode 1: Remote Control and Playback
