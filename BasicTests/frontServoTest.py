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
    print("Enter an angle between 0 and 180 to move the servo. Type 'exit' to quit.")

    while True:
        user_input = input("Enter angle: ")

        if user_input.lower() == 'exit':
            print("Exiting program.")
            break

        try:
            angle = float(user_input)
            set_servo_angle(angle)
        except ValueError:
            print("Invalid input. Please enter a numeric angle between 0 and 180, or type 'exit' to quit.")

except KeyboardInterrupt:
    print("\nProgram interrupted by user.")
finally:
    servo.close()
    print("Servo control cleaned up.")
