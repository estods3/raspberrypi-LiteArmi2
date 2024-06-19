#!/usr/bin/python2
import rospy
import RPi.GPIO as GPIO
import sys
from time import sleep
sys.path.append('../../lib')
from joint import joint


def main():
    #GPIO.setmode(GPIO.BCM) # BCM=13 is PWM1 channel on GPIO RPi3
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
    wristJoint = joint(joint_name="rb_joint_wrist", frequency=50, pin=12)

    # Calibration
    #------------
    # Motors are capable of duty cycle range: (5, 15)
    # However, robotic linkage constraints lower this capability for several of the sensors.
    wristJoint.setMin(1000, 0)
    wristJoint.setMax(2000, 95)
    wristJoint.setHome(0)

    # PROGRAM
    # -------
    wristJoint.start() #motor will start in home position
    #wristJoint.rotate(45, 0.2)
    wristJoint.standby()

    # ROS Spin
    # --------
    rospy.spin()

    # HOME AND STOP joint
    # --------------
    wristJoint.home()
    wristJoint.stop()

    # Cleanup
    # -------
    #GPIO.cleanup()

if __name__ == '__main__':
    main()
