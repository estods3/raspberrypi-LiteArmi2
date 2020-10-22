import RPi.GPIO as GPIO
from time import sleep
GPIO.setmode(GPIO.BCM)
GPIO.setup(33, GPIO.OUT)
pwm=GPIO.PWM(33, 100) # 50 Hz 

GPIO.setmode(GPIO.BCM)
GPIO.setup(33, GPIO.OUT)
pwm=GPIO.PWM(33, 100) # 50 Hz 

pwm.start(10) 
# "expects a pulse between 1 and 2 ms. 
# in other words, between a 5 and 10% duty cycle on a 50Hz waveform."
# https://www.pololu.com/product/1057/faqs
# https://dronebotworkshop.com/servo-motors-with-arduino/
sleep(1)

def SetAngle(angle):
	duty = angle / 18 + 2
	GPIO.output(33, True)
	pwm.ChangeDutyCycle(duty)
	sleep(1)
	GPIO.output(33, False)
	pwm.ChangeDutyCycle(0)

#SetAngle(90)
pwm.ChangeDutyCycle(5)
sleep(1)
pwm.stop()
GPIO.cleanup()
