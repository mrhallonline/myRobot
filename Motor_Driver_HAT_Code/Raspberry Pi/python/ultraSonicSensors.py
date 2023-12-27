
import RPi.GPIO as GPIO
import time
import threading
from robot import Robot

# Set the GPIO mode to BCM
GPIO.setmode(GPIO.BCM)
Motor = Robot()

try:
    # Create two threads for reading left and right sensors
    left_sensor_thread = threading.Thread(target=Motor.read_left_sensor, args=(2,))
    right_sensor_thread = threading.Thread(target=Motor.read_right_sensor, args=(2,))
    
    # Start both threads
    left_sensor_thread.start()
    right_sensor_thread.start()

    # Wait for both threads to finish (which they won't unless interrupted)
    left_sensor_thread.join()
    right_sensor_thread.join()

except KeyboardInterrupt:
    print("Measurement stopped by the user")
finally:
    GPIO.cleanup()  # Cleanup GPIO pins on exit
