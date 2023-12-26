import RPi.GPIO as GPIO
import time
import threading

# Define GPIO pins for the sensors
TRIG_PIN_LEFT = 27  # GPIO27
ECHO_PIN_LEFT = 17  # GPIO17

TRIG_PIN_RIGHT = 6  # GPIO18
ECHO_PIN_RIGHT = 5  # GPIO24

# Set the GPIO mode and set the pins as input/output
GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG_PIN_LEFT, GPIO.OUT)
GPIO.setup(ECHO_PIN_LEFT, GPIO.IN)
GPIO.setup(TRIG_PIN_RIGHT, GPIO.OUT)
GPIO.setup(ECHO_PIN_RIGHT, GPIO.IN)

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

def read_left_sensor():
    while True:
        distance_left = measure_distance(TRIG_PIN_LEFT, ECHO_PIN_LEFT)
        print(f"Left Distance: {distance_left:.2f} cm")
        time.sleep(1)  # Wait for 1 second between measurements

def read_right_sensor():
    while True:
        distance_right = measure_distance(TRIG_PIN_RIGHT, ECHO_PIN_RIGHT)
        print(f"Right Distance: {distance_right:.2f} cm")
        time.sleep(1)  # Wait for 1 second between measurements

try:
    # Create two threads for reading left and right sensors
    left_sensor_thread = threading.Thread(target=read_left_sensor)
    right_sensor_thread = threading.Thread(target=read_right_sensor)
    
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
