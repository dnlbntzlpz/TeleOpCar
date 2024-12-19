import pygame
import time

# control buttons:
#     shifter:
#         button 12 - 1
#         button 13 - 2
#         button 14 - 3
#         button 15 - 4
#         button 16 - 5
#         button 17 - 6
#     pedals:
#         axis 0 - wheel
#         axis 1 - accelerator
#         axis 2 - brake
#         axis 3 - clutch

# Initialize Pygame
pygame.init()

# Initialize Joystick
pygame.joystick.init()

# Find and Initialize the Racing Wheel
if pygame.joystick.get_count() == 0:
    print("No joystick detected. Please connect your racing wheel.")
    exit()

wheel = pygame.joystick.Joystick(0)
wheel.init()

print(f"Detected Racing Wheel: {wheel.get_name()}")

# Main Loop to Read Inputs
try:
    while True:
        # Handle Events
        pygame.event.pump()
        
        # Get Steering Wheel Axis Value
        steering_value = wheel.get_axis(0)  # Axis 0 is usually for the steering
        print(f"Steering Value: {steering_value:.2f}")
        
        # Get Pedals (Accelerator, Brake, Clutch)
        accelerator = wheel.get_axis(1)  # Axis 1 is the accelerator
        brake = wheel.get_axis(2)        # Axis 2 is the brake
        clutch = wheel.get_axis(3)       # Axis 3 is the clutch
        
        print(f"Pedals - Accelerator: {accelerator:.2f}, Brake: {brake:.2f}, Clutch: {clutch:.2f}")
        #print(f"Pedals - Accelerator: {accelerator:.2f}, Brake: {brake:.2f}")
        
        # Get Shifter Buttons
        for button in range(wheel.get_numbuttons()):
            if wheel.get_button(button):
                print(f"Button {button} Pressed")
        
        # Get D-Pad (HAT) Values
        hat = wheel.get_hat(0)
        print(f"D-Pad Hat: {hat}")

        time.sleep(1)
        
except KeyboardInterrupt:
    print("Exiting program.")

# Quit Pygame
pygame.quit()
