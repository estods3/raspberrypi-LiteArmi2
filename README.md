# 4 DoF Robotic Arm
3D-printed robotic arm using Raspberry Pi, ROS, and Python. Based on Open Sourced [LiteArm i2](https://www.thingiverse.com/thing:480446) Design.

<p align="center">
 <img src="https://github.com/estods3/raspberrypi-LiteArmi2/blob/master/docs/rosreplay.gif" alt="video" width="600"/>
</p>

## Hardware
The hardware for this robot consists of a 3D printed frame as well as a RaspberryPi with a custom "hat".
The link to the frame design as well as details on the circuitry and bill of materials are below.

### Frame
Based on Open Sourced [LiteArm i2](https://www.thingiverse.com/thing:480446) Design. 3D printed.

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
| Red LED | 1 | |
| 91ohm resistor | 1 | |
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
| NC | 5V | 
| NC | 5V |
| NC | NC |
| NC | NC |
| NC | Signal-Base (GPIO18) |
| NC | NC |
| NC | NC |
| NC | NC |
| NC | NC |
| NC | NC |
| NC | NC |
| Ground | NC |
| NC | NC |
| NC | NC |
| NC | NC |
| NC | Signal-Extension (GPIO13) |
| Signal-Wrist (GPIO12) | NC |
| NC | NC |
| Signal-LED | NC |
| Ground | NC |
</div>

The 20x2-pin female headers were placed on the underside of the perfoard to connect directly to the 20x2 set of male Raspberry Pi GPIO Pins.

NOTE: Position decoupling capacitors as close to 5V GPIO pins as possible.


## Software
The software for this robot was written in python for both the PC "command center" and the robot itself. All code is included in this single repository.

#### GPIO Pin Assignment
The following pin assignment was used for each servo (ensure the appropriate servo is connected to the right location on the 11x1 Male Header Connector.

<div align="center">
 
| Device | GPIO Pin |
|:----:|:----:|
| Base | GPIO18 |
| Extension | GPIO13 |
| Wrist | GPIO12 |
| LED | GPIO26 |

</div>


### ROS
This Robotic Arm uses Robotic Operating System (ROS) to transmit commands to the robot and also recieve live feedback on robot status and position.
<p align="center">
 <img src="https://github.com/estods3/raspberrypi-LiteArmi2/blob/master/rviz_interface.png" alt="video" width="800"/>
</p>

### Resources
https://github.com/AliShug/EvoArm
