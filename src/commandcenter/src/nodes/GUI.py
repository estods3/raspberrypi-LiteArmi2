#!/usr/bin/env python3
from tkinter import *
import rospy
from std_msgs.msg import Float32MultiArray, Int16
import sys,tty,termios

RCextpub = rospy.Publisher('cc_rb_joint_extension_set_angle', Float32MultiArray, queue_size=1)
modepub = rospy.Publisher('cc_rb_joints_mode', Int16, queue_size=1) # 1 - Program 1, 2 - Program 2, 3 - RC Mode, 4 - Home, 5 - Stop
RCmode = False

def RC_up_handler(event):
    r = rospy.Rate(3)
    # Publish Command
    homepub.publish(0)
    if(RCmode):
        print("FORWARD")
        RCextpub.publish()
        r.sleep()

def RC_down_handler(event):
    r = rospy.Rate(3)
    # Publish Command
    homepub.publish(0)
    if(RCmode):
        print("BACKWARD")
        RCextpub.publish(9)
        r.sleep()

def P1CallBack():
   global RCmode
   RCmode = False
   r = rospy.Rate(5)
   modepub.publish(1)
   print("--------- Program 1 ----------")
   r.sleep()

def P2CallBack():
   global RCmode
   RCmode = False
   r = rospy.Rate(5)
   print("--------- Program 2  ----------")
   modepub.publish(2)
   r.sleep()

def RCCallBack():
   r = rospy.Rate(5)
   global RCmode
   RCmode = True
   print("--------- In Remote Control Mode ----------")
   c = Float32MultiArray()
   c.data = [90, 0.1]
   RCextpub.publish(c)
   modepub.publish(3)
   r.sleep()

def RCBSCallBack():
   r = rospy.Rate(5)
   global RCmode
   RCmode = True
   print("--------- In Remote Control Mode: BASE ----------")
   c = Float32MultiArray()
   c.data = [90, 0.1]
   RCextpub.publish(c)
   homepub.publish(0)
   r.sleep()

def RCEXTCallBack():
   r = rospy.Rate(5)
   global RCmode
   RCmode = True
   print("--------- In Remote Control Mode: EXTENSION ----------")
   c = Float32MultiArray()
   c.data = [90, 0.1]
   RCextpub.publish(c)
   homepub.publish(0)
   r.sleep()

def RCWRCallBack():
   r = rospy.Rate(5)
   global RCmode
   RCmode = True
   print("--------- In Remote Control Mode: WRIST ----------")
   c = Float32MultiArray()
   c.data = [90, 0.1]
   RCextpub.publish(c)
   homepub.publish(0)
   r.sleep()

def StopCallBack():
   global RCmode
   RCmode = False
   r = rospy.Rate(5)
   print("---------        STOPPED         ----------")
   modepub.publish(5)
   r.sleep()

def HomeCallBack():
   global RCmode
   RCmode = False
   r = rospy.Rate(5)
   print("---------        HOMING         ----------")
   modepub.publish(4)
   r.sleep()

def main():
    window = Tk()
    window.title("Robot Arm Command Center")
    window.geometry("378x160")

    B_P1 = Button(window, text = "Program 1", width = 43, command = P1CallBack)
    B_P1.place(x = 5,y = 0)
    B_P2 = Button(window, text = "Program 2", width = 43, command = P2CallBack)
    B_P2.place(x = 5, y = 30)

    B_RC = Button(window, text = "Remote Control Mode", width = 17, command = RCCallBack)
    B_RC.place(x = 5, y = 60)
    B_BS = Button(window, text = "BASE", width = 5, command = RCBSCallBack)
    B_BS.place(x = 169, y = 60)
    B_EXT = Button(window, text = "EXT", width = 5, command = RCEXTCallBack)
    B_EXT.place(x = 237, y = 60)
    B_WR = Button(window, text = "WRIST", width = 5, command = RCWRCallBack)
    B_WR.place(x = 305, y = 60)

    B_HM = Button(window, text = "HOME", width = 43, command = HomeCallBack)
    B_HM.place(x = 5, y = 90)

    B_ES = Button(window, text = "STOP!", width = 43, command = StopCallBack)
    B_ES.place(x = 5, y = 120)

    frame = Frame(window, width = 10, height = 10)
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
