import piggio
pi = pigpio.pi()              # use defaults
if not pi.connected:
    exit()

pi.set_mode(18, pigpio.OUTPUT)

factory = PiGPIOFactory()

servo = AngularServo(18, min_angle=-45, max_angle=45, pin_factory=factory)
servo.angle = 0

pi.stop()

