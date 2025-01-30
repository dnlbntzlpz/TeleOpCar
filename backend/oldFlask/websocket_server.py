import asyncio
import websockets
import json
from motor_control import motor_forward, motor_backward, motor_stop

async def control_handler(websocket, path):
    print("Client connected")
    try:
        async for message in websocket:
            print(f"Received: {message}")
            data = json.loads(message)
            action = data.get("action")
            speed = data.get("speed", 1.0)  # Default to full speed (1.0)

            # Call motor control functions based on action
            if action == "forward":
                motor_forward(speed)
            elif action == "backward":
                motor_backward(speed)
            elif action == "stop":
                motor_stop()
    except websockets.ConnectionClosed:
        print("Client disconnected")
    finally:
        motor_stop()

def start_websocket_server():
    return websockets.serve(control_handler, "0.0.0.0", 8765)
