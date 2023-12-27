#!/usr/bin/python
# https://www.waveshare.com/wiki/Motor_Driver_HAT#Power
import RPi.GPIO as GPIO
import time
import threading
from gpiozero import DistanceSensor
from gpiozero.pins.pigpio import PiGPIOFactory
from PCA9685 import PCA9685

# Constants for direction
RMOTOR = 0
LMOTOR = 1
FORWARD = 'forward'
BACKWARD ='backward'

class Robot():
    def __init__(self):

        # Create a lock for thread-safe print statements
        self.print_lock = threading.Lock()

        # Motor control pin setup
        self.PWMA = 0
        self.AIN1 = 1
        self.AIN2 = 2
        self.PWMB = 5
        self.BIN1 = 3
        self.BIN2 = 4

        # Define GPIO pins for the sensors
        self.TRIG_PIN_LEFT = 27  # GPIO27
        self.ECHO_PIN_LEFT = 17  # GPIO17
        self.TRIG_PIN_RIGHT = 6  # GPIO18
        self.ECHO_PIN_RIGHT = 5  # GPIO24

        # Set the GPIO mode and set the pins as input/output
        GPIO.setup(self.TRIG_PIN_LEFT, GPIO.OUT)
        GPIO.setup(self.ECHO_PIN_LEFT, GPIO.IN)
        GPIO.setup(self.TRIG_PIN_RIGHT, GPIO.OUT)
        GPIO.setup(self.ECHO_PIN_RIGHT, GPIO.IN)

        # Initialize the PCA9685 module
        self.pwm = PCA9685(0x40, debug=False)
        self.pwm.setPWMFreq(50)

        # Create locks for sensor print synchronization
        print_lock = threading.Lock()

    def MotorRun(self, motor, direction, speed):
        # Ensure speed is within 0-100 range
        if not 0 <= speed <= 100:
            return

        # Set speed for motor
        self.pwm.setDutycycle(self.PWMA if motor == RMOTOR else self.PWMB, speed)

        # Set direction for motor
        if direction == FORWARD:
            self.pwm.setLevel(self.AIN1 if motor == RMOTOR else self.BIN1, 0)
            self.pwm.setLevel(self.AIN2 if motor == RMOTOR else self.BIN2, 1)
        elif direction == BACKWARD:
            self.pwm.setLevel(self.AIN1 if motor == RMOTOR else self.BIN1, 1)
            self.pwm.setLevel(self.AIN2 if motor == RMOTOR else self.BIN2, 0)

    def MotorStop(self, motor):
        # Stop specified motor
        self.pwm.setDutycycle(self.PWMA if motor == RMOTOR else self.PWMB, 0)

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

    def read_left_sensor(self, queue_len):
        gpio_factory_left = PiGPIOFactory()  # Create a separate factory for the left sensor
        sensor_l = DistanceSensor(echo=self.ECHO_PIN_LEFT, trigger=self.TRIG_PIN_LEFT, queue_len=queue_len, pin_factory=gpio_factory_left)
        while True:
            with self.print_lock:
                distance_left = sensor_l.distance * 100  # Convert to cm
                print(f"\rLeft Distance: {distance_left:.2f} cm |", end="")
            time.sleep(1)  # Wait for 1 second between measurements

    def read_right_sensor(self, queue_len):
        gpio_factory_right = PiGPIOFactory()  # Create a separate factory for the right sensor
        sensor_r = DistanceSensor(echo=self.ECHO_PIN_RIGHT, trigger=self.TRIG_PIN_RIGHT, queue_len=queue_len, pin_factory=gpio_factory_right)
        while True:
            with self.print_lock:
                distance_right = sensor_r.distance * 100  # Convert to cm
                print(f" Right Distance: {distance_right:.2f} cm", end="\n")
            time.sleep(1)  # Wait for 1 second between measurements