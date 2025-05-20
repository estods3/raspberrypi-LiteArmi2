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
    br_body = tf.TransformBroadcaster()
    br_rotator = tf.TransformBroadcaster()
    br_shoulder = tf.TransformBroadcaster()
    br_upperarm = tf.TransformBroadcaster()
    br_elbow = tf.TransformBroadcaster()
    br_forearm = tf.TransformBroadcaster()
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
        rotator_control_angle = math.radians(baseCurrentAngle)
        shoulder_control_angle = math.radians(extCurrentAngle)
        elbow_control_angle = math.radians(wristCurrentAngle)
        wrist_control_angle = 3.14/2 - elbow_control_angle

        # Fixed Frame to World Definition
        br_world.sendTransform((0, 0, 0),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     rospy.Time.now(), 'Fixed Frame', "World")

        # Link: Base (Fixed to World)
        br_base.sendTransform((0, 0, -1/100),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     rospy.Time.now(), 'World', "Base")

        # Joint: Rotator (Yaw)
        br_rotator.sendTransform((0, 0, 0),
                     tf.transformations.quaternion_from_euler(0, 0, rotator_control_angle),
                     rospy.Time.now(), 'Base', "Rotator")

        # Link: Body
        br_body.sendTransform((0, 0, 0),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     rospy.Time.now(), 'Rotator', "Body")

        # Joint: Shoulder (Pitch)
        br_shoulder.sendTransform((0, 0, -8.5/100),
                     tf.transformations.quaternion_from_euler(0, math.radians(70) - shoulder_control_angle, 0),
                     rospy.Time.now(), 'Body', "Shoulder")

        # Link: Upper Arm
        br_upperarm.sendTransform((16.0/100, 0, 0),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     rospy.Time.now(), 'Shoulder', "Upper Arm")

        # Joint: Elbow
        br_elbow.sendTransform((0, 0, 0),
                     tf.transformations.quaternion_from_euler(0, elbow_control_angle - 3.14, 0),
                     rospy.Time.now(), 'Upper Arm', "Elbow")

        # Link: Fore Arm
        br_forearm.sendTransform((17.0/100, 0, 0),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     rospy.Time.now(), 'Elbow', "Fore Arm")
        
        # Joint: Wrist
        br_wrist.sendTransform((0, 0, 0),
                     tf.transformations.quaternion_from_euler(0, wrist_control_angle, 0),
                     rospy.Time.now(), "Fore Arm", "Wrist")

        # Link: End Effector
        br_endeffector.sendTransform((5.0/100, 0, 0),
                     tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(), 'Wrist', "End Effector")

    # Cleanup
    # -------
    GPIO.cleanup()

if __name__ == '__main__':
    main()
