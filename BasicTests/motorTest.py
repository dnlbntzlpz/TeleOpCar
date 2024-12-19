from gpiozero import PWMOutputDevice, OutputDevice
from time import sleep

# Define GPIO pins for Motor 1
M1_IN1 = PWMOutputDevice(12)  # Motor 1, IN1: Speed control (PWM)
M1_IN2 = OutputDevice(13)     # Motor 1, IN2: Direction control (GPIO)

# Define GPIO pins for Motor 2
M2_IN1 = PWMOutputDevice(18)  # Motor 2, IN1: Speed control (PWM)
M2_IN2 = OutputDevice(19)     # Motor 2, IN2: Direction control (GPIO)

try:
    while True:
        # Motor 1: Forward with gradual speed increase
        print("Motor 1 moving forward...")
        M1_IN2.off()  # Direction: Forward
        for speed in range(0, 101, 10):  # Gradually increase speed
            M1_IN1.value = speed / 100.0  # PWM duty cycle (0.0 to 1.0)
            print(f"Motor 1 Speed: {speed}%")
            sleep(0.5)
        
        # Motor 2: Forward at constant speed
        print("Motor 2 moving forward...")
        M2_IN2.off()  # Direction: Forward
        M2_IN1.value = 0.5  # 50% speed
        sleep(2)

        # Stop motors
        print("Stopping motors...")
        M1_IN1.off()
        M2_IN1.off()
        sleep(1)

        # Motor 1: Reverse at gradual speed increase
        print("Motor 1 moving backward...")
        M1_IN2.on()  # Direction: Reverse
        for speed in range(0, 101, 10):  # Gradually increase speed
            M1_IN1.value = speed / 100.0
            print(f"Motor 1 Speed: {speed}%")
            sleep(0.5)

        # Motor 2: Reverse at constant speed
        print("Motor 2 moving backward...")
        M2_IN2.on()  # Direction: Reverse
        M2_IN1.value = 0.75  # 75% speed
        sleep(2)

        # Stop motors
        print("Stopping motors...")
        M1_IN1.off()
        M2_IN1.off()
        sleep(2)

except KeyboardInterrupt:
    print("Test stopped by user.")

finally:
    M1_IN1.off()
    M1_IN2.off()
    M2_IN1.off()
    M2_IN2.off()
    print("GPIO cleanup done.")
