import asyncio
import websockets
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from dotenv import load_dotenv
import os

# Load environment variables from .env file
load_dotenv()

# Get port from environment variables
WEBSOCKET_PORT = int(os.getenv("WEBSOCKET_PORT", 8765))  # Default to 8765

class WebSocketROS2Bridge(Node):
    def __init__(self):
        super().__init__('websocket_ros2_bridge')

        # ROS2 publishers
        self.steering_pub = self.create_publisher(Float32, '/servo_command', 10)
        self.motor_pub = self.create_publisher(Float32, '/motor_command', 10)

    async def handle_connection(self, websocket, path):
        self.get_logger().info("WebSocket client connected")
        try:
            async for message in websocket:
                data = json.loads(message)

                # Publish steering and motor commands
                if 'steering_angle' in data:
                    self.steering_pub.publish(Float32(data=data['steering_angle']))
                if 'motor_speed' in data:
                    self.motor_pub.publish(Float32(data=data['motor_speed']))

                self.get_logger().info(f"Received: {data}")
        except websockets.ConnectionClosed:
            self.get_logger().info("WebSocket client disconnected")


    async def run_server(self):
        server = await websockets.serve(self.handle_connection, '0.0.0.0', WEBSOCKET_PORT)
        self.get_logger().info(f"WebSocket server started on port {WEBSOCKET_PORT}")
        await server.wait_closed()

def main(args=None):
    rclpy.init(args=args)
    node = WebSocketROS2Bridge()

    loop = asyncio.get_event_loop()
    try:
        loop.run_until_complete(node.run_server())
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
