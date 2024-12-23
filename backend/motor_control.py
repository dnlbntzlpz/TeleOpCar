from gpiozero import PWMOutputDevice, DigitalOutputDevice

# Motor 1 (Left Motor) setup
motor1_pwm = PWMOutputDevice(12)  # PWM pin
motor1_dir = DigitalOutputDevice(13)  # Direction pin

# Motor 2 (Right Motor) setup
motor2_pwm = PWMOutputDevice(18)  # PWM pin
motor2_dir = DigitalOutputDevice(19)  # Direction pin

# Motor control functions
def motor_forward(speed=1.0):
    """Move motors forward at the given speed (0.0 to 1.0)."""
    motor1_dir.off()  # Set direction for forward
    motor2_dir.off()
    motor1_pwm.value = speed
    motor2_pwm.value = speed

def motor_backward(speed=1.0):
    """Move motors backward at the given speed (0.0 to 1.0)."""
    motor1_dir.on()  # Set direction for backward
    motor2_dir.on()
    motor1_pwm.value = speed
    motor2_pwm.value = speed

def motor_stop():
    """Stop both motors."""
    motor1_pwm.off()
    motor2_pwm.off()
