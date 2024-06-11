import rospy
import RPi.GPIO as GPIO
from time import sleep
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32, Int16, String
from datetime import datetime

class joint:
    def __init__(self, joint_name, frequency, pin):

        self.name = joint_name
        self.mode = 3 #default RC mode

        # ROS NETWORK
        print("JOINT %s: ROS node created" %(self.name))
        rospy.init_node(joint_name + "_node", anonymous=True)
        print("JOINT %s: Set Angle Subscriber created" %(self.name))
        self.nextpossub = rospy.Subscriber("cc_" + joint_name + "_set_angle", Float32MultiArray, self.rotateROS, queue_size=1)
        print("JOINT %s: Publisher created" %(self.name))
        self.currentpospub = rospy.Publisher(joint_name + "_current_angle", Float32, queue_size=1)
        print("JOINT %s: Mode Subscriber created" %(self.name))
        self.modesub = rospy.Subscriber("cc_rb_joints_mode", Int16, self.setMode, queue_size=1) # 1 - Program 1, 2 - Program 2, 3 - RC Mode, 4 - Home, 5 - Stop
        self.r = rospy.Rate(5)

        # I/O
        self.pwmPin = pin
        GPIO.setup(self.pwmPin, GPIO.OUT)
        self.motor=GPIO.PWM(self.pwmPin, frequency) # communicated with motor (50Hz)

        # "expects a pulse between 1 and 2 ms.
        # in other words, between a 5 and 10% duty cycle on a 50Hz waveform."
        # https://www.pololu.com/product/1057/faqs
        # https://dronebotworkshop.com/servo-motors-with-arduino/
        # ------------------------------------------------------------------
        # For motors used in this project, 5 to 15% duty cycle is the range.
        self.minPos = (0, 0) # tuple (DutyCycle, Angle)
        self.maxPos = (0, 0) # tuple (DutyCycle, Angle)
        self.currentPos = (0, 0) # tuple (DutyCycle, Angle)
        self.homePos = (0, 0) # tuple (DutyCycle, Angle)

    def standby(self):
        while(not rospy.is_shutdown()):
            c = Float32(self.getCurrent("Angle"))
            self.currentpospub.publish(c)
            now = datetime.now()
            current_time = now.strftime("%S")
            print("JOINT %s: @%ss Current Position is %s degrees" %(self.name, current_time, self.getCurrent("Angle")))
            self.r.sleep()

    def setMode(self, data):
        self.mode = data.data
        print("JOINT %s: Mode set to %s" %(self.name, self.mode))
        if(self.mode == 4):
             self.home()
        # TODO implement stop=5 and other modes

    def start(self):
        if(self.name == "rb_joint_ext"):
            sleep(1.5)
        if(self.name == "rb_joint_base"):
            sleep(3)
        self.motor.start(self.getHome("DC"))
        self.currentPos = self.homePos
        c = Float32(self.getCurrent("Angle"))
        self.currentpospub.publish(c)
        print("JOINT %s: Started at Home (%s, %s)" %(self.name, self.currentPos[0], self.currentPos[1]))

    def setMin(self, dc, angle):
        self.minPos = (dc, angle)

    def setMax(self, dc, angle):
        self.maxPos = (dc, angle)

    def setHome(self, angle):
        self.homePos = (self.getDCfromAngle(angle), angle)

    def getMin(self, string):
        return self.minPos[0] if string is "DC" else self.minPos[1]

    def getMax(self, string):
        return self.maxPos[0] if string is "DC" else self.maxPos[1]

    def getCurrent(self, string):
        return self.currentPos[0] if string is "DC" else self.currentPos[1]

    def getHome(self, string):
        return self.homePos[0] if string is "DC" else self.homePos[1]

    def home(self):
        if(self.name == "rb_joint_ext"):
            sleep(1.5)
        if(self.name == "rb_joint_base"):
            sleep(3)
        print("JOINT %s: Homing" %(self.name))
        self.rotate(self.getHome("Angle"), 0.3)
        print("JOINT %s: Homing Done" %(self.name))

    def stop(self):
        print("JOINT %s: Stopping Motor" %(self.name))
        self.motor.stop()
        print("JOINT %s: Motor Stopped" %(self.name))

    def getDCfromAngle(self, angle):
        if(angle > self.getMax("Angle")):
            print("JOINT %s: Warning - Angle (%s) is greater than max (%s). Setting angle to max." %(self.name, angle, self.getMax("Angle")))
            angle = self.getMax("Angle")
        if(angle < self.getMin("Angle")):
            print("JOINT %s: Warning - Angle (%s) is less than min (%s). Setting angle to min." %(self.name, angle, self.getMin("Angle")))
            angle = self.getMin("Angle")
        dc = ((self.getMax("DC") - self.getMin("DC")) * (angle - self.getMin("Angle")))/float(self.getMax("Angle") - self.getMin("Angle")) + self.getMin("DC")
        return dc

    def rotateROS(self, data):
        args = data.data
        if(self.mode == 3):
            print("JOINT %s: ROS Request Recieved. Rotating to Angle (%s)." %(self.name, args[0]))
            self.rotate(args[0], args[1])
            print("JOINT %s: ROS Request Complete." %(self.name))
        c = Float32(self.getCurrent("Angle"))
        self.currentpospub.publish(c)
        self.r.sleep()

    def rotate(self, endPosAngle, speedDelayTime):
        print("JOINT %s: Starting Rotation to %s degrees from %s degrees" %(self.name, endPosAngle, self.getCurrent("Angle")))
        endPos = self.getDCfromAngle(endPosAngle)
        nextPos = self.getCurrent("DC")
        increment=1
        while(abs(nextPos - endPos) >= 0.5):
            if(abs(nextPos - endPos) <= 3):
                increment = 0.5
            if(nextPos < endPos):
                nextPos = nextPos + increment
                #print("rotating + direction")
            else:
                nextPos = nextPos - increment
                #print("rotating - direction")
            #print("current=%s, end=%s" %(nextPos, endPos))
            self.motor.ChangeDutyCycle(int(nextPos))
            sleep(speedDelayTime)
            self.currentPos = (nextPos, endPosAngle)
            c = Float32(self.getCurrent("Angle"))
            self.currentpospub.publish(c)
        print("JOINT %s: Rotation Complete" %(self.name))
