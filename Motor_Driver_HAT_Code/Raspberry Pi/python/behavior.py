#!/usr/bin/python

import robot
from PCA9685 import PCA9685
from time import sleep

# Constants for direction
RMOTOR = 0
LMOTOR = 1
TESTSPEED = 95
TESTTIME = 1

# Test code
# Usage example:
Motor = robot.MotorDriver()

# Forward motion
Motor.Forward(TESTSPEED)
sleep(TESTTIME)
Motor.Backward(TESTSPEED)
sleep(TESTTIME)
Motor.Right(50, 75)
sleep(TESTTIME)
Motor.Left(75, 50)
sleep(TESTTIME)

# Stop
Motor.MotorStop(RMOTOR)
Motor.MotorStop(LMOTOR)