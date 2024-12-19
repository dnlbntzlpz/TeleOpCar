import pygame

# Initialize Pygame Joystick module
pygame.init()
pygame.joystick.init()

# Check for connected joysticks
joystick_count = pygame.joystick.get_count()
print(f"Number of Joysticks: {joystick_count}")

if joystick_count > 0:
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"Joystick Name: {joystick.get_name()}")

    try:
        print("---------------------------")
        while True:
            pygame.event.pump()
            for i in range(joystick.get_numaxes()):
                axis_value = joystick.get_axis(i)
                print(f"Axis {i}: {axis_value}")
            for i in range(joystick.get_numbuttons()):
                button_value = joystick.get_button(i)
                print(f"Button {i}: {button_value}")
            pygame.time.wait(100)  # Adjust for smoother input reading
    except KeyboardInterrupt:
        print("Exiting...")
else:
    print("No joystick found!")
