import socket
import json
import time
import pygame

# Server configuration
SERVER_IP = "192.168.10.183"  # Replace with your Raspberry Pi's IP
PORT = 8765                   # Port to connect to

# Initialize pygame for joystick input
pygame.init()
pygame.joystick.init()

# Ensure at least one joystick is connected
if pygame.joystick.get_count() == 0:
    print("No joystick connected! Please connect a racing wheel controller.")
    exit()

# Initialize the first joystick
joystick = pygame.joystick.Joystick(0)
joystick.init()
print(f"Using joystick: {joystick.get_name()}")

def get_racing_wheel_input():
    """
    Captures inputs from the racing wheel using pygame.
    Returns a dictionary of shifter and pedal states.
    """
    pygame.event.pump()  # Process event queue to get updated input states

    # Buttons (shifter)
    shifter_buttons = {
        "button_1": joystick.get_button(12),
        "button_2": joystick.get_button(13),
        "button_3": joystick.get_button(14),
        "button_4": joystick.get_button(15),
        "button_5": joystick.get_button(16),
        "button_6": joystick.get_button(17),
    }

    # Axes (pedals and wheel)
    pedals_and_wheel = {
        "wheel": joystick.get_axis(0),         # Steering wheel (-1.0 to 1.0)
        "accelerator": (joystick.get_axis(1) + 1) / 2,  # Normalize to 0.0 to 1.0
        "brake": (joystick.get_axis(2) + 1) / 2,        # Normalize to 0.0 to 1.0
        "clutch": (joystick.get_axis(3) + 1) / 2,       # Normalize to 0.0 to 1.0
    }

    return {"shifter": shifter_buttons, "pedals": pedals_and_wheel}

# Create a socket
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

try:
    print(f"Connecting to {SERVER_IP}:{PORT}...")
    client_socket.connect((SERVER_IP, PORT))
    print("Connected to the server.")

    while True:
        # Capture racing wheel inputs
        inputs = get_racing_wheel_input()

        # Serialize inputs to JSON and send to the server
        message = json.dumps(inputs)
        client_socket.sendall(message.encode())
        print(f"Sent: {message}")

        # Wait for 1 second before sending the next message
        time.sleep(1)

except KeyboardInterrupt:
    print("\nExiting program.")
finally:
    print("Closing the connection.")
    client_socket.close()
    pygame.quit()
