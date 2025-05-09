#!/usr/bin/env python3.8
import rospy
import math
import tf
from std_msgs.msg import Float32, Float32MultiArray, Int16

extCurrentAngle = 10
baseCurrentAngle = 0
wristCurrentAngle = 0
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

def tf_tree():
    rospy.init_node('rb_main_node', anonymous=True)
    br_world = tf.TransformBroadcaster()
    br_base = tf.TransformBroadcaster()
    br_extension = tf.TransformBroadcaster()
    br_extension2 = tf.TransformBroadcaster()
    br_wrist = tf.TransformBroadcaster()
    br_endeffector = tf.TransformBroadcaster()
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
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
        br_extension.sendTransform((0.75, 0, -0.5),
                     tf.transformations.quaternion_from_euler(0, initial_offset, 0),
                     rospy.Time.now(), 'Base', "Extension1")
        br_extension2.sendTransform((5, 0, 0),
                     tf.transformations.quaternion_from_euler(0, extension_control_angle, 0),
                     rospy.Time.now(), 'Extension1', "Extension2")

        initial_offset = math.radians(90)
        br_wrist.sendTransform((5, 0, 0),
                     tf.transformations.quaternion_from_euler(0, initial_offset - extension_control_angle, 0),
                     rospy.Time.now(), 'Extension2', "Wrist")
        initial_offset = -3.14/3
        br_endeffector.sendTransform((0.5, 0, 0),
                     tf.transformations.quaternion_from_euler(0, initial_offset + wrist_control_angle, 0), rospy.Time.now(), 'Wrist', "End Effector")

        rate.sleep()

if __name__ == '__main__':
    try:
        tf_tree()
    except rospy.ROSInterruptException:
        pass
