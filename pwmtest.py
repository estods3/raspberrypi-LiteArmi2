import RPi.GPIO as GPIO
from time import sleep
GPIO.setmode(GPIO.BCM) # BCM=13 is PWM1 channel on GPIO RPi3
pin=13
GPIO.setup(pin, GPIO.OUT)
pwm=GPIO.PWM(pin, 50) # 50 Hz, communicated with motor

pwm.start(15)
# "expects a pulse between 1 and 2 ms. 
# in other words, between a 5 and 10% duty cycle on a 50Hz waveform."
# https://www.pololu.com/product/1057/faqs
# https://dronebotworkshop.com/servo-motors-with-arduino/
sleep(1)
i = 0
while i < 10:
	pwm.ChangeDutyCycle(5)
	sleep(1)
	pwm.ChangeDutyCycle(10)
	sleep(1)
	i = i + 1

pwm.stop()
GPIO.cleanup()
