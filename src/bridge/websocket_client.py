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
    pygame.event.pump()  # Process event queue to get updated input states
    pedals_and_wheel = {
        "wheel": joystick.get_axis(0),          # Steering wheel (-1.0 to 1.0)
        "accelerator": (joystick.get_axis(1) + 1) / 2,  # Normalize to 0.0 to 1.0
        "brake": (joystick.get_axis(2) + 1) / 2,        # Normalize to 0.0 to 1.0
    }
    return {"pedals": pedals_and_wheel}

client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

try:
    print(f"Connecting to {SERVER_IP}:{PORT}...")
    client_socket.connect((SERVER_IP, PORT))
    print("Connected to the server.")

    while True:
        inputs = get_racing_wheel_input()
        message = json.dumps(inputs)
        client_socket.sendall(message.encode())
        print(f"Sent: {message}")
        time.sleep(1)
except KeyboardInterrupt:
    print("\nExiting program.")
finally:
    print("Closing the connection.")
    client_socket.close()
    pygame.quit()
