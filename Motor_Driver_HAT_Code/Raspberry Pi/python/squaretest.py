#!/usr/bin/python

import robot
from PCA9685 import PCA9685
from time import sleep

# Constants for direction
RMOTOR = 0
LMOTOR = 1
TESTSPEED = 90
TESTTIME = 2.25

# Test code
# Usage example:
Motor = robot.MotorDriver()

# Forward motion
Motor.Forward('forward', TESTSPEED)
sleep(TESTTIME)

# Stop
Motor.MotorStop(RMOTOR)
Motor.MotorStop(LMOTOR)