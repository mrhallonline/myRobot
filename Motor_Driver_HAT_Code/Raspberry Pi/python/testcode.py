#!/usr/bin/python

import robot
from PCA9685 import PCA9685
from time import sleep

# Constants for direction
RMOTOR = 0
LMOTOR = 1
TESTSPEED = 100
TESTTIME = 1

# Test code
# Usage example:
Motor = robot.MotorDriver()

# Forward motion
Motor.Forward('forward', TESTSPEED)
sleep(TESTTIME)
Motor.Backward('backward', TESTSPEED)
sleep(TESTTIME)
Motor.LeftMotor('forward', 50)
Motor.RightMotor('forward', 50)
sleep(TESTTIME)
Motor.LeftMotor('forward', 50)
Motor.RightMotor('backward', 50)
sleep(TESTTIME)
Motor.LeftMotor('backward', 50)
Motor.RightMotor('forward', 50)
sleep(TESTTIME)

# Stop
Motor.MotorStop(RMOTOR)
Motor.MotorStop(LMOTOR)