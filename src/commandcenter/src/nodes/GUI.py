#!/usr/bin/env python3
from tkinter import *
import rospy
from std_msgs.msg import Float32MultiArray
import sys,tty,termios

RCextpub = rospy.Publisher('cc_rb_joint_extension_set_angle', Float32MultiArray, queue_size=1)
RCmode = False

def RC_left_handler(event):
    r = rospy.Rate(3)
    # Publish Command
    if(RCmode):
        print("LEFT")
        RCextpub.publish(6)
        r.sleep()

def RC_right_handler(event):
    r = rospy.Rate(3)
    # Publish Command
    if(RCmode):
        print("RIGHT")
        RCextpub.publish(7)
        r.sleep()

def RC_up_handler(event):
    r = rospy.Rate(3)
    # Publish Command
    if(RCmode):
        print("FORWARD")
        RCextpub.publish(8)
        r.sleep()

def RC_down_handler(event):
    r = rospy.Rate(3)
    # Publish Command
    if(RCmode):
        print("BACKWARD")
        RCextpub.publish(9)
        r.sleep()

def LFCallBack():
   global RCmode
   RCmode = False
   r = rospy.Rate(5)
   print("--------- Program 1 ----------")
   CCpub.publish(1)
   r.sleep()

def WSCallBack():
   global RCmode
   RCmode = False
   r = rospy.Rate(5)
   print("--------- Program 2  ----------")
   CCpub.publish(2)
   r.sleep()

def RCCallBack():
   r = rospy.Rate(5)
   global RCmode
   RCmode = True
   print("--------- In Remote Control Mode ----------")
   CCpub.publish(3)
   r.sleep()

def StopCallBack():
   global RCmode
   RCmode = False
   r = rospy.Rate(5)
   print("---------        STOPPED         ----------")
   CCpub.publish(4)
   r.sleep()

def main():
    window = Tk()
    window.title("Robot Arm Command Center")
    window.geometry("330x125")
    B_LF = Button(window, text = "Program 1", width = 37, command = LFCallBack)
    B_LF.place(x = 5,y = 0)
    B_WS = Button(window, text = "Program 2", width = 37, command = WSCallBack)
    B_WS.place(x = 5, y = 30)
    B_RC = Button(window, text = "Remote Control Mode", width = 37, command = RCCallBack)
    B_RC.place(x = 5, y = 60)
    B_ES = Button(window, text = "STOP!", width = 37, command = StopCallBack)
    B_ES.place(x = 5, y = 90)
    frame = Frame(window, width = 5, height = 5)
    #window.bind('<Left>', RC_left_handler)
    #window.bind('<Right>', RC_right_handler)
    window.bind('<Up>', RC_up_handler)
    window.bind('<Down>', RC_down_handler)
    frame.pack()
    window.mainloop()

if __name__ == '__main__':
    try:
        rospy.init_node('pc_command_center_node', anonymous=True)
        main()
    except(KeyboardInterrupt,SystemExit):
        print("---- Exiting ----")
