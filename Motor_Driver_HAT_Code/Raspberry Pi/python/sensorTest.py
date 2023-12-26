import RPi.GPIO as GPIO
import time
import threading
from gpiozero import DistanceSensor
from gpiozero.pins.pigpio import PiGPIOFactory

# Set the GPIO mode to BCM
GPIO.setmode(GPIO.BOARD)

print("Prep pins")
sensor_l= DistanceSensor(echo=11, trigger=13, queue_len=2)
sensor_r = DistanceSensor(echo=29, trigger=31, queue_len=2)

while True:
    print("Left: {l}, Right: {r}".format(
        l=sensor_l.distance * 100,
        r=sensor_r.distance * 100))

