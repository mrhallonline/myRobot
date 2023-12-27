#!/usr/bin/python

from robot import Robot
from time import sleep

# Constants for direction and test parameters

TESTSPEEDRight = 85
TESTSPEEDLeft = 85
TESTTIME = 1


# Instantiatie the Robot class
Motor = Robot()

# Forward motion
Motor.Forward(TESTSPEEDRight, TESTSPEEDLeft)
sleep(TESTTIME)

#Backward motion
Motor.Backward(TESTSPEEDRight, TESTSPEEDLeft)
sleep(TESTTIME)


# Stop both motors
Motor.MotorStop(Motor.RMOTOR)
Motor.MotorStop(Motor.LMOTOR)