#!/usr/bin/python2
import rospy
import RPi.GPIO as GPIO
import sys
from time import sleep
from std_msgs.msg import Bool

def main():
    GPIO.setmode(GPIO.BCM) 
    GPIO.setwarnings(False)
    led_pin = 26
    GPIO.setup(led_pin,GPIO.OUT) #GPIO26
    status = True

    pub = rospy.Publisher('rb_led_heartbeat', Bool , queue_size=10)
    rospy.init_node('rb_main_node', anonymous=True)
    rate = rospy.Rate(5) # 5hz
    while not rospy.is_shutdown():
        heartbeat_msg = status
        if(status):
            GPIO.output(led_pin,GPIO.HIGH)
        else:
            GPIO.output(led_pin,GPIO.LOW)

        status = not status
        pub.publish(heartbeat_msg)
        rate.sleep()

    # Cleanup
    # -------
    GPIO.cleanup()

if __name__ == '__main__':
    main()
