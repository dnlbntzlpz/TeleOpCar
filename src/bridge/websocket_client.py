import pygame
import asyncio
import websockets
import json
from dotenv import load_dotenv
import os

# Load environment variables from .env file
load_dotenv()

# Get Raspberry Pi IP and port from environment variables
RASPBERRY_PI_IP = os.getenv("RASPBERRY_PI_IP", "127.0.0.1")  # Default to localhost
WEBSOCKET_PORT = int(os.getenv("WEBSOCKET_PORT", 8765))       # Default to 8765

# WebSocket URI
uri = f"ws://{RASPBERRY_PI_IP}:{WEBSOCKET_PORT}"

async def send_commands():
    # Connect to the WebSocket server
    async with websockets.connect(uri) as websocket:
        pygame.init()
        joystick = pygame.joystick.Joystick(0)
        joystick.init()

        print(f"Detected Racing Wheel: {joystick.get_name()}")

        while True:
            pygame.event.pump()

            # Read steering axis
            steering = joystick.get_axis(0)  # Axis 0 for steering
            steering_angle = steering * 90   # Map -1 to 1 -> -90° to 90°

            # Read pedals
            accelerator = -joystick.get_axis(1)  # Axis 1 for accelerator (invert for positive value)
            brake = -joystick.get_axis(2)        # Axis 2 for brake (invert for positive value)

            # Combine accelerator and brake into a single motor speed value
            motor_speed = max(0, accelerator) - max(0, brake)

            # Create the command message
            command = {
                'steering_angle': steering_angle,
                'motor_speed': motor_speed
            }

            # Send the command as JSON
            await websocket.send(json.dumps(command))
            print(f"Sent: {command}")

            await asyncio.sleep(0.05)  # 20 Hz update rate

if __name__ == "__main__":
    try:
        asyncio.run(send_commands())
    except KeyboardInterrupt:
        print("Exiting...")
