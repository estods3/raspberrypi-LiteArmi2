#!/usr/bin/python2
import rospy
import RPi.GPIO as GPIO
import sys
from time import sleep
sys.path.append('../../lib')
from joint import joint


def main():
    GPIO.setmode(GPIO.BCM) # BCM=13 is PWM1 channel on GPIO RPi3
    #         ROBOT DIAGRAM
    #
    #   ext_motor    /\
    #           >   /  \ < elbow_motor
    #              /    \
    #           __/      \_
    #          |__|
    #              ^base_motor

    # ROBOT CONFIGURATION
    # -------------------
    extJoint = joint(joint_name="rb_joint_extension", frequency=50, pin=13)
    #baseJoint = joint(frequency=50, pin=18)
    #elbowJoint = joint(frequency=50, pin=12)

    # Calibration
    #------------
    # Motors are capable of duty cycle range: (5, 15)
    # However, robotic linkage constraints lower this capability for several of the sensors.
    #baseJoint.setMin(5, -45)
    #baseJoint.setMax(15, 45)
    #baseJoint.setHome(0)

    extJoint.setMin(7.5, 45) # @0 degrees Servo Horn is horizontal (parallel) with ground plane
    extJoint.setMax(10, 90) # @90 degrees Servo Horn is verticle (perpendicular) with ground plane
    extJoint.setHome(90)

    #elbowJoint.setMin(8, 0)
    #elbowJoint.setMax(10, 20)
    #elbowJoint.setHome(0)

    # PROGRAM
    # -------
    extJoint.start() #motor will start in home position
    extJoint.rotate(45, 0.2)

    # ROS Spin
    # --------
    rospy.spin()

    # HOME AND STOP joint
    # --------------
    extJoint.home()
    extJoint.stop()

    # Cleanup
    # -------
    GPIO.cleanup()

if __name__ == '__main__':
    main()
