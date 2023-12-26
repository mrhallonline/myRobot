#!/usr/bin/python
# https://www.waveshare.com/wiki/Motor_Driver_HAT#Power
import RPi.GPIO as GPIO
from PCA9685 import PCA9685
import time
import threading
from gpiozero import DistanceSensor
from gpiozero.pins.pigpio import PiGPIOFactory

# Constants for direction
RMOTOR = 0
LMOTOR = 1
FORWARD = 'forward'
BACKWARD ='backward'
TESTSPEED = 100
TESTTIME = 1

# Set the GPIO mode to BCM
GPIO.setmode(GPIO.BCM)

# Define GPIO pins for the sensors
TRIG_PIN_LEFT = 27  # GPIO27
ECHO_PIN_LEFT = 17  # GPIO17

TRIG_PIN_RIGHT = 6  # GPIO18
ECHO_PIN_RIGHT = 5  # GPIO24

# Set the GPIO mode and set the pins as input/output
GPIO.setup(TRIG_PIN_LEFT, GPIO.OUT)
GPIO.setup(ECHO_PIN_LEFT, GPIO.IN)
GPIO.setup(TRIG_PIN_RIGHT, GPIO.OUT)
GPIO.setup(ECHO_PIN_RIGHT, GPIO.IN)

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

    def Forward(self, rightMotorSpeed, leftMotorSpeed):
        print("forward")
        self.MotorRun(RMOTOR, 'forward', rightMotorSpeed)
        self.MotorRun(LMOTOR, 'forward', leftMotorSpeed)

    def Backward(self, rightMotorSpeed, leftMotorSpeed):
        print("backward")
        self.MotorRun(RMOTOR, 'backward', rightMotorSpeed)
        self.MotorRun(LMOTOR, 'backward', leftMotorSpeed)

    # Individual motor control including speed and direction, should be placed together
    def LeftMotor(self, direction, speed): # self, "backward"/"forward", 0-100
        print("left motor engaged")
        self.MotorRun(LMOTOR, direction, speed)
    def RightMotor(self, direction, speed):  # self, "backward"/"forward", 0-100
        print("right motor engaged")
        self.MotorRun(RMOTOR, direction, speed)