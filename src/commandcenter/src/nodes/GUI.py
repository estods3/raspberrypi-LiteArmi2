#!/usr/bin/env python3
from tkinter import *
import rospy
from std_msgs.msg import Float32, Float32MultiArray, Int16
import sys,tty,termios

extCurrentAngle = 0
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

RCextpub = rospy.Publisher('cc_rb_joint_extension_set_angle', Float32MultiArray, queue_size=1)
RCextsub = rospy.Subscriber('rb_joint_extension_current_angle', Float32, RCextCurrentSub, queue_size=1)
RCbasepub = rospy.Publisher('cc_rb_joint_base_set_angle', Float32MultiArray, queue_size=1)
RCbasesub = rospy.Subscriber('rb_joint_base_current_angle', Float32, RCbaseCurrentSub, queue_size=1)
RCwristpub = rospy.Publisher('cc_rb_joint_wrist_set_angle', Float32MultiArray, queue_size=1)
RCwristsub = rospy.Subscriber('rb_joint_wrist_current_angle', Float32, RCwristCurrentSub, queue_size=1)

modepub = rospy.Publisher('cc_rb_joints_mode', Int16, queue_size=1) # 1 - Program 1, 2 - Program 2, 3 - RC Mode, 4 - Home, 5 - Stop
RCmode = False
RCjoint = 1 # 1 - base, 2 - ext, 3 - wrist

def RC_up_handler(event):
    global RCmode
    global extCurrentAngle
    global RCjoint
    r = rospy.Rate(3)
    # Publish Command
    if(RCmode):
        print("%s: Moving 15 degrees in Positive(+) Direction" %(RCjoint))
        a = Float32MultiArray()
        a.data = [baseCurrentAngle, 0.2]
        b = Float32MultiArray()
        b.data = [extCurrentAngle, 0.2]
        c = Float32MultiArray()
        c.data = [wristCurrentAngle, 0.2]

        if(RCjoint==1):
            a.data = [baseCurrentAngle + 15.0, 0.2]
        elif(RCjoint==2):
            b.data = [extCurrentAngle + 15.0, 0.2]
        else:
            c.data = [wristCurrentAngle + 15.0, 0.2]

        RCbasepub.publish(a)
        RCextpub.publish(b)
        RCwristpub.publish(c)

        r.sleep()

def RC_down_handler(event):
    global RCmode
    global extCurrentAngle
    global RCjoint
    r = rospy.Rate(3)
    # Publish Command
    if(RCmode):
        print("%s: Moving 15 degrees in Negative(-) Direction" %(RCjoint))
        a = Float32MultiArray()
        a.data = [baseCurrentAngle, 0.2]
        b = Float32MultiArray()
        b.data = [extCurrentAngle, 0.2]
        c = Float32MultiArray()
        c.data = [wristCurrentAngle, 0.2]

        if(RCjoint==1):
            a.data = [baseCurrentAngle - 15.0, 0.2]
        elif(RCjoint==2):
            b.data = [extCurrentAngle - 15.0, 0.2]
        else:
            c.data = [wristCurrentAngle - 15.0, 0.2]

        RCbasepub.publish(a)
        RCextpub.publish(b)
        RCwristpub.publish(c)

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
    modepub.publish(3)
    r.sleep()

def RCBSCallBack():
    global RCmode
    global RCjoint
    RCmode = True
    print("--------- In Remote Control Mode: BASE ----------")
    RCjoint = 1

def RCEXTCallBack():
    global RCmode
    global RCjoint
    RCmode = True
    print("--------- In Remote Control Mode: EXTENSION ----------")
    RCjoint = 2

def RCWRCallBack():
    global RCmode
    global RCjoint
    RCmode = True
    print("--------- In Remote Control Mode: WRIST ----------")
    RCjoint = 3

def StopCallBack():
   global RCmode
   RCmode = False
   r = rospy.Rate(5)
   print("--------- STOPPED ----------")
   modepub.publish(5)
   r.sleep()

def HomeCallBack():
   global RCmode
   RCmode = False
   r = rospy.Rate(5)
   print("--------- HOMING ----------")
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
