#!/usr/bin/python

import robot
from PCA9685 import PCA9685
from time import sleep

# Constants for direction
RMOTOR = 0
LMOTOR = 1
TESTSPEEDRight = 85
TESTSPEEDLeft = 85
TESTTIME = 1

# Test code
# Usage example:
Motor = robot.MotorDriver()

# Forward motion
Motor.Forward(TESTSPEEDRight, TESTSPEEDLeft)
sleep(TESTTIME)
Motor.Backward(TESTSPEEDRight, TESTSPEEDLeft)
sleep(TESTTIME)


# Stop
Motor.MotorStop(RMOTOR)
Motor.MotorStop(LMOTOR)