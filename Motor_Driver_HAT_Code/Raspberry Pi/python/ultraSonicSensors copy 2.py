import RPi.GPIO as GPIO
import time

# Define GPIO pins for the sensor
TRIG_PIN = 27  # GPIO18
ECHO_PIN = 17  # GPIO24

# Set the GPIO mode and set the pins as input/output
GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG_PIN, GPIO.OUT)
GPIO.setup(ECHO_PIN, GPIO.IN)

def measure_distance():
    # Set the trigger pin LOW initially
    GPIO.output(TRIG_PIN, False)
    time.sleep(0.2)  # Allow time for the sensor to settle
    
    # Send a 10us pulse on the trigger pin to trigger the sensor
    GPIO.output(TRIG_PIN, True)
    time.sleep(0.00001)
    GPIO.output(TRIG_PIN, False)
    
    # Wait for the echo pin to go high, and then start the timer
    while GPIO.input(ECHO_PIN) == 0:
        pulse_start = time.time()
    
    # Wait for the echo pin to go low, and then stop the timer
    while GPIO.input(ECHO_PIN) == 1:
        pulse_end = time.time()
    
    # Calculate the duration of the pulse and convert it to distance (cm)
    pulse_duration = pulse_end - pulse_start
    distance_cm = (pulse_duration * 34300) / 2  # Speed of sound is approximately 343 m/s
    
    return distance_cm

try:
    while True:
        distance = measure_distance()
        print(f"Distance: {distance:.2f} cm")
        time.sleep(1)  # Wait for 1 second between measurements

except KeyboardInterrupt:
    print("Measurement stopped by the user")
finally:
    GPIO.cleanup()  # Cleanup GPIO pins on exit
