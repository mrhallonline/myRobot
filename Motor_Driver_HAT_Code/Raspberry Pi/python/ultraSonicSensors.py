
import RPi.GPIO as GPIO
import time
import threading
from robot import Robot

# Set the GPIO mode to BCM
GPIO.setmode(GPIO.BCM)
Robot = Robot()

try:
    # Create two threads for reading left and right sensors
    right_sensor_thread = threading.Thread(target=Robot.read_right_sensor, args=(2,))
    left_sensor_thread = threading.Thread(target=Robot.read_left_sensor, args=(2,))

    
    # Start both threads
    right_sensor_thread.start()
    left_sensor_thread.start()


    # Wait for both threads to finish (which they won't unless interrupted)
    right_sensor_thread.join()   
    left_sensor_thread.join()


except KeyboardInterrupt:
    print("Measurement stopped by the user")
finally:
    GPIO.cleanup()  # Cleanup GPIO pins on exit
