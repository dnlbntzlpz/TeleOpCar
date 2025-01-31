import json
from channels.generic.websocket import AsyncWebsocketConsumer
from .ros_interface import ROS2Interface

ros2_interface = ROS2Interface()

class ControlConsumer(AsyncWebsocketConsumer):
    async def connect(self):
        await self.accept()

    async def disconnect(self, close_code):
        pass

    async def receive(self, text_data):
        data = json.loads(text_data)
        action = data.get("action")

        if action == "forward":
            ros2_interface.publish_brake(1.0)  # Full brake
            ros2_interface.publish_accelerator(1.0)
        elif action == "backward":
            ros2_interface.publish_brake(1.0)  # Full brake
            ros2_interface.publish_accelerator(-.9)
        elif action == "stop":
            #ros2_interface.publish_accelerator(1.0)  # Full brake
            ros2_interface.publish_brake(0.0)  # Full brake
        elif action == "left":
            ros2_interface.publish_steering(45.0)
        elif action == "right":
            ros2_interface.publish_steering(135.0)
        elif action == "center":
            ros2_interface.publish_steering(90.0)  # Center steering at 90 degrees

        # Send confirmation to frontend
        await self.send(text_data=json.dumps({"status": "success", "action": action}))
