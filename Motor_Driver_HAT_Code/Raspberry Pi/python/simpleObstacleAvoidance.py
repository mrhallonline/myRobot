#!/usr/bin/python

import threading
from robot import Robot
from time import sleep

# Constants for test parameters
TESTSPEED = 50
OBSTACLE_DISTANCE = 30  # Distance in cm to consider an obstacle
TESTTIME = 1  # Time the robot moves in a direction before checking distance again

# Instantiate the Robot class
robot = Robot()

def obstacle_avoidance():
    while True:
        left_distance = robot.read_left_sensor(50)
        right_distance = robot.read_right_sensor(50)

        if left_distance > OBSTACLE_DISTANCE and right_distance > OBSTACLE_DISTANCE:
            # Path is clear
            robot.Forward(TESTSPEED, TESTSPEED)
        elif left_distance < OBSTACLE_DISTANCE:
            # Obstacle on the left, turn right
            robot.RightMotor('forward', TESTSPEED)
            robot.LeftMotor('backward', TESTSPEED)
        elif right_distance < OBSTACLE_DISTANCE:
            # Obstacle on the right, turn left
            robot.LeftMotor('forward', TESTSPEED)
            robot.RightMotor('backward', TESTSPEED)
        else:
            # Obstacle in both directions, stop or reverse
            robot.MotorStop(Robot.RMOTOR)
            robot.MotorStop(Robot.LMOTOR)

        sleep(TESTTIME)



# Start sensor reading threads (assumes these methods update distance attributes)
left_sensor_thread = threading.Thread(target=robot.read_left_sensor, args=(2,))
right_sensor_thread = threading.Thread(target=robot.read_right_sensor, args=(2,))

left_sensor_thread.start()
right_sensor_thread.start()

# Start the obstacle avoidance behavior
try:
    obstacle_avoidance()
except KeyboardInterrupt:
    GPIO.cleanup()  # Cleanup GPIO pins on exit
    print("Obstacle avoidance stopped by the user")
finally:
    Motor.MotorStop(Motor.RMOTOR)
    Motor.MotorStop(Motor.LMOTOR)
    GPIO.cleanup()  # Cleanup GPIO pins on exit
