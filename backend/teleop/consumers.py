import json
from channels.generic.websocket import AsyncWebsocketConsumer
from .motor_control import motor_forward, motor_backward, motor_stop, steer_left, steer_right, steer_center

class ControlConsumer(AsyncWebsocketConsumer):
    async def connect(self):
        await self.accept()
        print("WebSocket connection established.")

    async def disconnect(self, close_code):
        print("WebSocket connection closed.")
        motor_stop()

    async def receive(self, text_data):
        data = json.loads(text_data)
        action = data.get("action")
        speed = data.get("speed", 1.0)

        if action == "forward":
            motor_forward(speed)
        elif action == "backward":
            motor_backward(speed)
        elif action == "stop":
            motor_stop()
        elif action == "left":
            steer_left()
        elif action == "right":
            steer_right()
        elif action == "center":
            steer_center()

        # Send a confirmation back to the frontend
        await self.send(text_data=json.dumps({"status": "success", "action": action}))
