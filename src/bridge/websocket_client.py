import socket
import json
import time
import pygame
from dotenv import load_dotenv
import os

# Load environment variables from .env file
load_dotenv()

SERVER_IP = os.getenv("RASPBERRY_PI_IP", "127.0.0.1")  # Default to localhost
PORT = int(os.getenv("SOCKET_PORT", 8765))            # Default to port 8765

# Initialize pygame for joystick input
pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    print("No joystick connected! Please connect a racing wheel controller.")
    exit()

joystick = pygame.joystick.Joystick(0)
joystick.init()
print(f"Using joystick: {joystick.get_name()}")

def get_racing_wheel_input():
    # Default values when the controller is idle
    wheel, accelerator, brake = 0.0, 1.0, 1.0

    pygame.event.pump()  # Process event queue to get updated input states

    # Force polling of all axes
    try:
        axis_wheel = joystick.get_axis(0)
        axis_accelerator = joystick.get_axis(1)
        axis_brake = joystick.get_axis(2)

        # If the joystick is reporting default "0.0" for all axes, retain the manually set defaults
        if axis_wheel != 0.0 or axis_accelerator != 0.0 or axis_brake != 0.0:
            wheel = float(axis_wheel)
            accelerator = float(axis_accelerator)
            brake = float(axis_brake)
    except Exception as e:
        print(f"Error reading joystick inputs: {e}")
        # Keep default values in case of error

    return {"pedals": {"wheel": wheel, "accelerator": accelerator, "brake": brake}}

client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

try:
    print(f"Connecting to {SERVER_IP}:{PORT}...")
    client_socket.connect((SERVER_IP, PORT))
    print("Connected to the server.")

    while True:
        # Get inputs with enforced default values
        inputs = get_racing_wheel_input()

        # Serialize inputs to JSON and send to the server
        message = json.dumps(inputs)
        client_socket.sendall(message.encode())
        print(f"Sent: {message}")

        # Send updates every 50 ms (20 Hz)
        time.sleep(0.05)

except KeyboardInterrupt:
    print("\nExiting program.")
finally:
    print("Closing the connection.")
    client_socket.close()
    pygame.quit()
