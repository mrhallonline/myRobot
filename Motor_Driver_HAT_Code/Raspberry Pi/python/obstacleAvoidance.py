#!/usr/bin/python

import threading
from robot import Robot
import RPi.GPIO as GPIO
from time import sleep

# ... other imports ...

GPIO.setmode(GPIO.BCM)  # Set the GPIO mode


# Constants for test parameters
MAX_SPEED = 75
MIN_SPEED = 30
OBSTACLE_THRESHOLD = 50  # Distance in cm at which to start slowing down
OBSTACLE_CLOSE = 20      # Distance in cm for minimal speed or stop
TESTTIME = 0.1  # Time the robot moves in a direction before checking distance again

def calculate_speed(distance):
    if distance > OBSTACLE_THRESHOLD:
        return MAX_SPEED
    elif distance < OBSTACLE_CLOSE:
        return MIN_SPEED
    else:
        # Linearly scale speed based on distance
        return MIN_SPEED + (MAX_SPEED - MIN_SPEED) * (distance - OBSTACLE_CLOSE) / (OBSTACLE_THRESHOLD - OBSTACLE_CLOSE)

def obstacle_avoidance():
    while True:
        left_distance = Motor.get_left_distance()
        right_distance = Motor.get_right_distance()

        left_speed = calculate_speed(left_distance)
        right_speed = calculate_speed(right_distance)

        if left_distance > OBSTACLE_THRESHOLD and right_distance > OBSTACLE_THRESHOLD:
            # Path is clear
            Motor.Forward(right_speed, left_speed)
        elif left_distance < OBSTACLE_THRESHOLD:
            # Obstacle on the left, slow down left motor
            Motor.RightMotor('forward', right_speed)
            Motor.LeftMotor('forward', left_speed)
        elif right_distance < OBSTACLE_THRESHOLD:
            # Obstacle on the right, slow down right motor
            Motor.LeftMotor('forward', left_speed)
            Motor.RightMotor('forward', right_speed)
        else:
            # Very close obstacles, stop or reverse
            Motor.MotorStop(Robot.RMOTOR)
            Motor.MotorStop(Robot.LMOTOR)

        sleep(TESTTIME)

# Instantiate the Robot class
Motor = Robot()

# Start sensor reading threads (assumes these methods update distance attributes)
left_sensor_thread = threading.Thread(target=Motor.read_left_sensor, args=(2,))
right_sensor_thread = threading.Thread(target=Motor.read_right_sensor, args=(2,))

left_sensor_thread.start()
right_sensor_thread.start()

# Start the obstacle avoidance behavior
try:
    obstacle_avoidance()
except KeyboardInterrupt:
    print("Obstacle avoidance stopped by the user")
finally:
    Motor.MotorStop(Motor.RMOTOR)
    Motor.MotorStop(Motor.LMOTOR)
    GPIO.cleanup()  # Cleanup GPIO pins on exit
