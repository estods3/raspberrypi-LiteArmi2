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
    baseJoint = joint(joint_name="rb_joint_base", frequency=50, pin=18)

    # Calibration
    #------------
    # Motors are capable of duty cycle range: (5, 15)
    # However, robotic linkage constraints lower this capability for several of the sensors.
    baseJoint.setMin(5, -45)
    baseJoint.setMax(15, 45)
    baseJoint.setHome(0)

    # PROGRAM
    # -------
    baseJoint.start() #motor will start in home position
    #baseJoint.rotate(15, 0.2)
    baseJoint.standby()

    # ROS Spin
    # --------
    rospy.spin()

    # HOME AND STOP joint
    # --------------
    baseJoint.home()
    baseJoint.stop()

    # Cleanup
    # -------
    GPIO.cleanup()

if __name__ == '__main__':
    main()
