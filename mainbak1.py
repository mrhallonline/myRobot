#!/usr/bin/python

# https://www.waveshare.com/wiki/Motor_Driver_HAT#Power

from PCA9685 import PCA9685
import time

# Constants for direction
RMOTOR = 0
LMOTOR = 1
FORWARD = 'forward'
BACKWARD ='backward'
TESTSPEED = 100
TESTTIME = 1

# Initialize the PCA9685 module
pwm = PCA9685(0x40, debug=False)
pwm.setPWMFreq(50)

class MotorDriver():
    def __init__(self):
        # Motor control pin setup
        self.PWMA = 0
        self.AIN1 = 1
        self.AIN2 = 2
        self.PWMB = 5
        self.BIN1 = 3
        self.BIN2 = 4

    def MotorRun(self, motor, direction, speed):
        # Ensure speed is within 0-100 range
        if not 0 <= speed <= 100:
            return

        # Set speed for motor
        pwm.setDutycycle(self.PWMA if motor == RMOTOR else self.PWMB, speed)

        # Set direction for motor
        if direction == FORWARD:
            pwm.setLevel(self.AIN1 if motor == RMOTOR else self.BIN1, 0)
            pwm.setLevel(self.AIN2 if motor == RMOTOR else self.BIN2, 1)
        elif direction == BACKWARD:
            pwm.setLevel(self.AIN1 if motor == RMOTOR else self.BIN1, 1)
            pwm.setLevel(self.AIN2 if motor == RMOTOR else self.BIN2, 0)


    def MotorStop(self, motor):
        # Stop specified motor
        pwm.setDutycycle(self.PWMA if motor == RMOTOR else self.PWMB, 0)

    def Forward(self, speed):
        print("forward")
        self.MotorRun(RMOTOR, 'forward', speed)
        self.MotorRun(LMOTOR, 'forward', speed)

    def Backward(self, speed):
        print("backward")
        self.MotorRun(RMOTOR, 'backward', speed)
        self.MotorRun(LMOTOR, 'backward', speed)

    def Left(self, rightMotorSpeed, leftMotorSpeed):
        print("left")
        self.MotorRun(RMOTOR, 'backward', rightMotorSpeed)
        self.MotorRun(LMOTOR, 'forward', leftMotorSpeed)

    def Right(self, rightMotorSpeed, leftMotorSpeed):
        print("right")
        self.MotorRun(RMOTOR, 'forward', rightMotorSpeed)
        self.MotorRun(LMOTOR, 'backward', leftMotorSpeed)

# Test code
# Usage example:
Motor = MotorDriver()

# Forward motion
Motor.Forward(TESTSPEED)
time.sleep(TESTTIME)

# Backwards motion
Motor.Backward(TESTSPEED)
time.sleep(TESTTIME)

# Right turn
Motor.Right(50, 75)
time.sleep(TESTTIME)

# Left turn
Motor.Left(75, 50)
time.sleep(TESTTIME)

# Stop
Motor.MotorStop(RMOTOR)
Motor.MotorStop(LMOTOR)