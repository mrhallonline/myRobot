#!/usr/bin/python

from robot import Robot
from time import sleep

# Constants for direction and test parameters

TESTSPEEDRight = 85
TESTSPEEDLeft = 85
TESTTIME = 1


# Instantiatie the Robot class
robot = Robot()

# Forward motion
robot.Forward(TESTSPEEDRight, TESTSPEEDLeft)
sleep(TESTTIME)

#Backward motion
robot.Backward(TESTSPEEDRight, TESTSPEEDLeft)
sleep(TESTTIME)


# Stop both motors
robot.MotorStop(Robot.RMOTOR)
robot.MotorStop(Robot.LMOTOR)