import RPi.GPIO as GPIO
from time import sleep

class joint:
    def __init__(self, frequency, pin):
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

    def start(self):
        self.motor.start(self.getHome("DC"))
        self.currentPos = self.homePos
        print("JOINT: Started at Home (%s, %s)" %(self.currentPos[0], self.currentPos[1]))

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
        print("JOINT: Homing")
        self.rotate(self.getHome("Angle"), 0.3)
        print("JOINT: Homing Done")

    def stop(self):
        self.motor.stop()

    def getDCfromAngle(self, angle):
        dc = ((self.getMax("DC") - self.getMin("DC")) * (angle - self.getMin("Angle")))/float(self.getMax("Angle") - self.getMin("Angle")) + self.getMin("DC")
        return dc

    def rotate(self, endPosAngle, speedDelayTime):
        print("Starting Rotation")
        print(endPosAngle)
        endPos = self.getDCfromAngle(endPosAngle)
        nextPos = self.getCurrent("DC")
        increment=1
        print("current=%s, end=%s" %(nextPos, endPos))
        while(abs(nextPos - endPos) >= 0.5):
            if(abs(nextPos - endPos) <= 3):
                increment = 0.5
            if(nextPos < endPos):
                nextPos = nextPos + increment
                print("rotating + direction")
            else:
                nextPos = nextPos - increment
                print("rotating - direction")
            print("current=%s, end=%s" %(nextPos, endPos))
            self.motor.ChangeDutyCycle(int(nextPos))
            sleep(speedDelayTime)
        self.currentPos = (nextPos, endPosAngle)

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
    #

    # ROBOT CONFIGURATION
    # -------------------
    extJoint = joint(frequency=50, pin=13)
    #baseJoint = joint(frequency=50, pin=18)
    #elbowJoint = joint(frequency=50, pin=12)

    # Calibration
    #------------
    # Motors are capable of duty cycle range: (5, 15)
    # However, robotic linkage constraints lower this capability for several of the sensors.
    #baseJoint.setMin(5, -45)
    #baseJoint.setMax(15, 45)
    #baseJoint.setHome(0)

    extJoint.setMin(5, 0) # @0 Servo Horn is horizontal (parallel) with ground plane
    extJoint.setMax(10, 90) # @90 Servo Horn is verticle (perpendicular) with ground plane
    extJoint.setHome(90)

    #elbowJoint.setMin(8, 0)
    #elbowJoint.setMax(10, 20)
    #elbowJoint.setHome(0)

    # PROGRAM
    # -------
    extJoint.start() #motor will start in home position
    #elbowJoint.start() #motor will start in home position
    #baseJoint.start() #motor will start in home position

#    print("ROTATING to 0 Degrees")
#    extJoint.rotate(30, 0.2)
#    max=10
#    x=0
#    while(x < max):
#        sleep(0.05)
#        x=x+0.05
    print("ROTATING to 45 Degrees")
    extJoint.rotate(45, 0.2)

    # HOME the Robot
    # --------------
    max=10
    x=0
    while(x < max):
        sleep(0.05)
        x=x+0.05
    extJoint.home()
    #elbowJoint.home()
    #baseJoint.home()

    # Stop all Motors
    # -----------------
    extJoint.stop()
    #elbowJoint.stop()
    #baseJoint.stop()

    # Cleanup
    # -------
    GPIO.cleanup()

if __name__ == '__main__':
    main()
