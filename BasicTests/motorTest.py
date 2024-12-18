import RPi.GPIO as GPIO
from time import sleep

# Define GPIO pins for IN1 and IN2
IN1 = 17  # Replace with your actual GPIO pin for IN1
IN2 = 27  # Replace with your actual GPIO pin for IN2

# Setup GPIO
GPIO.setmode(GPIO.BCM)  # Using BCM pin numbering
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)

try:
    while True:
        # Test motor in one direction
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)
        print("Motor running in one direction...")
        sleep(2)

        # Test motor in the opposite direction
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.HIGH)
        print("Motor running in opposite direction...")
        sleep(2)

        # Stop motor briefly
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.LOW)
        print("Motor stopped...")
        sleep(2)

except KeyboardInterrupt:
    print("Test stopped by user.")

finally:
    GPIO.cleanup()
    print("GPIO cleanup done.")
