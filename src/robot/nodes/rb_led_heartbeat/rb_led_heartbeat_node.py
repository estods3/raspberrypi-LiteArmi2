#!/usr/bin/python2
import rospy
import RPi.GPIO as GPIO
import sys
from time import sleep
import math
import tf
from std_msgs.msg import Float32, Float32MultiArray, Int16, Bool

extCurrentAngle = 10
baseCurrentAngle = 0
wristCurrentAngle = 75
def RCextCurrentSub(data):
    global extCurrentAngle
    extCurrentAngle = data.data

def RCbaseCurrentSub(data):
    global baseCurrentAngle
    baseCurrentAngle = data.data

def RCwristCurrentSub(data):
    global wristCurrentAngle
    wristCurrentAngle = data.data

RCextsub = rospy.Subscriber('rb_joint_extension_current_angle', Float32, RCextCurrentSub, queue_size=1)
RCbasesub = rospy.Subscriber('rb_joint_base_current_angle', Float32, RCbaseCurrentSub, queue_size=1)
RCwristsub = rospy.Subscriber('rb_joint_wrist_current_angle', Float32, RCwristCurrentSub, queue_size=1)

def main():
    rospy.init_node('rb_main_node', anonymous=True)

    # LED
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    led_pin = 26
    GPIO.setup(led_pin,GPIO.OUT) #GPIO26
    status = True

    pub = rospy.Publisher('rb_led_heartbeat', Bool , queue_size=10)

    # TF
    br_world = tf.TransformBroadcaster()
    br_base = tf.TransformBroadcaster()
    br_extension = tf.TransformBroadcaster()
    br_extension2 = tf.TransformBroadcaster()
    br_wrist = tf.TransformBroadcaster()
    br_endeffector = tf.TransformBroadcaster()
    rate = rospy.Rate(5) # 5hz

    while not rospy.is_shutdown():
        # Update LED
        heartbeat_msg = status
        if(status):
            GPIO.output(led_pin,GPIO.HIGH)
        else:
            GPIO.output(led_pin,GPIO.LOW)

        status = not status
        pub.publish(heartbeat_msg)
        rate.sleep()

        # Update TF Tree
        global wristCurrentAngle
        global baseCurrentAngle
        global extCurrentAngle
        base_control_angle = math.radians(baseCurrentAngle)
        extension_control_angle = math.radians(extCurrentAngle)
        wrist_control_angle = math.radians(wristCurrentAngle)

        br_world.sendTransform((0, 0, 0),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     rospy.Time.now(), 'Fixed Frame', "World")

        br_base.sendTransform((0, 0, -1),
                     tf.transformations.quaternion_from_euler(0, 0, base_control_angle),
                     rospy.Time.now(), 'World', "Base")

        initial_offset = math.radians(-45)
        br_extension.sendTransform((0.75/100, 0, -0.5/100),
                     tf.transformations.quaternion_from_euler(0, initial_offset, 0),
                     rospy.Time.now(), 'Base', "Shoulder")
        br_extension2.sendTransform((16.0/100, 0, 0),
                     tf.transformations.quaternion_from_euler(0, math.radians(60) - extension_control_angle, 0),
                     rospy.Time.now(), 'Shoulder', "Elbow")

        initial_offset = math.radians(90)
        br_wrist.sendTransform((17.0/100, 0, 0),
                     tf.transformations.quaternion_from_euler(0, wrist_control_angle - 3.14, 0),
                     rospy.Time.now(), 'Elbow', "Wrist")
        initial_offset = math.radians(-120)
        br_endeffector.sendTransform((5.0/100, 0, 0),
                     tf.transformations.quaternion_from_euler(0, 3.14/2 - wrist_control_angle, 0), rospy.Time.now(), 'Wrist', "End Effector")

    # Cleanup
    # -------
    GPIO.cleanup()

if __name__ == '__main__':
    main()
