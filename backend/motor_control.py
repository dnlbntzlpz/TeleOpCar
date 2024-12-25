from gpiozero import PWMOutputDevice, DigitalOutputDevice, PWMOutputDevice
from time import sleep

# Motor 1 (Left Motor) setup
motor1_pwm = PWMOutputDevice(13)  # PWM pin
motor1_dir = DigitalOutputDevice(16)  # Direction pin

# Motor 2 (Right Motor) setup
motor2_pwm = PWMOutputDevice(18)  # PWM pin
motor2_dir = DigitalOutputDevice(23)  # Direction pin

# Use GPIO12 for the servo control
SERVO_PWM_PIN = 12  # GPIO pin
servo = PWMOutputDevice(SERVO_PWM_PIN, frequency=50)  # 50Hz for standard servos

# Motor control functions
def motor_forward(speed=0.9):
    """Move motors forward at the given speed (0.0 to 1.0)."""
    motor1_dir.off()  # Set direction for forward
    motor2_dir.off()
    motor1_pwm.value = speed
    motor2_pwm.value = speed

def motor_backward(speed=0.9):
    """Move motors backward at the given speed (0.0 to 1.0)."""
    motor1_dir.on()  # Set direction for backward
    motor2_dir.on()
    motor1_pwm.value = speed
    motor2_pwm.value = speed

def motor_stop():
    print("Stopping all motors")
    motor1_pwm.off()
    motor2_pwm.off()
    motor1_dir.off()
    motor2_dir.off()

# Servo control functions
last_servo_angle = None
DEADBAND_THRESHOLD = 1  # µsec equivalent for your calculations

# Servo control functions remain unchanged
def set_servo_angle(angle):
    global last_servo_angle

    if angle < 0 or angle > 180:
        print(f"Invalid angle: {angle}. Must be between 0 and 180 degrees.")
        return

    if last_servo_angle is not None and abs(angle - last_servo_angle) <= DEADBAND_THRESHOLD:
        print(f"Angle {angle}° is within the deadband of {last_servo_angle}°, no update needed.")
        return

    pulse_width_us = 500 + (angle / 180) * 2000  # Map angle to 500-2500µs
    duty_cycle = pulse_width_us / 20000  # Convert to duty cycle (20ms period for 50Hz)
    servo.value = round(duty_cycle, 4)  # Limit to 4 decimal places
    last_servo_angle = angle

    print(f"Set servo to {angle}° (Duty Cycle: {duty_cycle:.4f})")
    #sleep(0.5)  # Allow time for the servo to move



def steer_left():
    """Turn the servo to the leftmost position."""
    print("Steering Left")
    set_servo_angle(10)

def steer_right():
    """Turn the servo to the rightmost position."""
    print("Steering Right")
    set_servo_angle(170)

def steer_center():
    """Center the servo."""
    print("Steering Center")
    set_servo_angle(90)