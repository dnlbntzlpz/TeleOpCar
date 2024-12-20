from gpiozero import PWMOutputDevice
from time import sleep

# Use a hardware PWM-capable GPIO pin
SERVO_PWM_PIN = 12  # Replace with your actual GPIO PWM pin

# Initialize PWM output device
servo = PWMOutputDevice(SERVO_PWM_PIN, frequency=50)  # Standard 50Hz for servos

# Function to map angle to duty cycle
def set_servo_angle(angle):
    """
    Moves the servo to a specific angle.
    Pulse width range:
    - 500µs (0°) -> 2.5% duty cycle
    - 1500µs (neutral) -> 7.5% duty cycle
    - 2500µs (max, 180°/270°) -> 12.5% duty cycle
    """
    if angle < 0 or angle > 180:
        print(f"Invalid angle: {angle}. Must be between 0 and 180 degrees.")
        return

    pulse_width_us = 500 + (angle / 180) * 2000  # Map angle to 500-2500µs range
    duty_cycle = pulse_width_us / 20000  # Convert pulse width to duty cycle (20ms period for 50Hz)

    servo.value = duty_cycle
    print(f"Angle: {angle}°, Pulse Width: {pulse_width_us}µs, Duty Cycle: {duty_cycle:.4f}")
    sleep(1)  # Allow time for the servo to move

try:
    print("Servo will move continuously from 0° to 180° and back. Press Ctrl+C to stop.")

    while True:
        # Move from 0 to 180 degrees
        for angle in range(0, 181, 10):  # Increment by 10 degrees
            set_servo_angle(angle)

        # Move from 180 to 0 degrees
        for angle in range(180, -1, -10):  # Decrement by 10 degrees
            set_servo_angle(angle)

except KeyboardInterrupt:
    print("\nProgram interrupted by user.")
finally:
    servo.close()
    print("Servo control cleaned up.")
