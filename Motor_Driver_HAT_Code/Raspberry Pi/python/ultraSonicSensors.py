import robot
import RPi.GPIO as GPIO
import time
import threading
from gpiozero import DistanceSensor
from gpiozero.pins.pigpio import PiGPIOFactory

# Set the GPIO mode to BCM
GPIO.setmode(GPIO.BCM)

# Define GPIO pins for the sensors
TRIG_PIN_LEFT = 27  # GPIO27
ECHO_PIN_LEFT = 17  # GPIO17

TRIG_PIN_RIGHT = 6  # GPIO18
ECHO_PIN_RIGHT = 5  # GPIO24

# Create locks for synchronization
print_lock = threading.Lock()

def measure_distance(trigger_pin, echo_pin):
    # Set the trigger pin LOW initially
    GPIO.output(trigger_pin, False)
    time.sleep(0.2)  # Allow time for the sensor to settle
    
    # Send a 10us pulse on the trigger pin to trigger the sensor
    GPIO.output(trigger_pin, True)
    time.sleep(0.00001)
    GPIO.output(trigger_pin, False)
    
    # Wait for the echo pin to go high, and then start the timer
    while GPIO.input(echo_pin) == 0:
        pulse_start = time.time()
    
    # Wait for the echo pin to go low, and then stop the timer
    while GPIO.input(echo_pin) == 1:
        pulse_end = time.time()
    
    # Calculate the duration of the pulse and convert it to distance (cm)
    pulse_duration = pulse_end - pulse_start
    distance_cm = (pulse_duration * 34300) / 2  # Speed of sound is approximately 343 m/s
    
    return distance_cm


def read_left_sensor(queue_len):
    gpio_factory_left = PiGPIOFactory()  # Create a separate factory for the left sensor
    sensor_l = DistanceSensor(echo=ECHO_PIN_LEFT, trigger=TRIG_PIN_LEFT, queue_len=queue_len, pin_factory=gpio_factory_left)
    while True:
        with print_lock:
            distance_left = sensor_l.distance * 100  # Convert to cm
            print(f"\rLeft Distance: {distance_left:.2f} cm |", end="")
        time.sleep(1)  # Wait for 1 second between measurements

def read_right_sensor(queue_len):
    gpio_factory_right = PiGPIOFactory()  # Create a separate factory for the right sensor
    sensor_r = DistanceSensor(echo=ECHO_PIN_RIGHT, trigger=TRIG_PIN_RIGHT, queue_len=queue_len, pin_factory=gpio_factory_right)
    while True:
        with print_lock:
            distance_right = sensor_r.distance * 100  # Convert to cm
            print(f" Right Distance: {distance_right:.2f} cm", end="\n")
        time.sleep(1)  # Wait for 1 second between measurements

try:
    gpio_factory = PiGPIOFactory()

    # You can set the queue_len parameter here for each sensor
    queue_len_left = 2  # Customize as needed
    queue_len_right = 2  # Customize as needed

    # Create two threads for reading left and right sensors
    left_sensor_thread = threading.Thread(target=read_left_sensor, args=(queue_len_left,))
    right_sensor_thread = threading.Thread(target=read_right_sensor, args=(queue_len_right,))
    
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
