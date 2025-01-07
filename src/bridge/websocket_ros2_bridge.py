import socket
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from dotenv import load_dotenv
import os

# Load environment variables from .env file
load_dotenv()

class SocketROS2Bridge(Node):
    def __init__(self):
        super().__init__('socket_ros2_bridge')

        # ROS2 publishers
        self.steering_pub = self.create_publisher(Float32, '/servo_command', 10)
        self.motor_pub = self.create_publisher(Float32, '/motor_command', 10)
        self.accelerator_pub = self.create_publisher(Float32, '/accelerator_command', 10)
        self.brake_pub = self.create_publisher(Float32, '/brake_command', 10)

        # Server configuration from environment variables
        self.HOST = os.getenv("HOST", "0.0.0.0")  # Default to listen on all network interfaces
        self.PORT = int(os.getenv("SOCKET_PORT", 8765))  # Default to port 8765

        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind((self.HOST, self.PORT))
        self.server_socket.listen(5)  # Allow up to 5 queued connections

        self.get_logger().info(f"Server started on {self.HOST}:{self.PORT}, waiting for connections...")

    def handle_client(self, conn, addr):
        """Handles an individual client connection."""
        self.get_logger().info(f"Connected by {addr}")
        try:
            with conn:
                while rclpy.ok():
                    data = conn.recv(1024)
                    if not data:
                        break  # Client disconnected
                    message = data.decode()
                    self.get_logger().info(f"Received: {message}")

                    try:
                        # Parse JSON data
                        inputs = json.loads(message)

                        # Extract raw values
                        steering_input = inputs.get("pedals", {}).get("wheel", 0)  # Raw -1.0 to 1.0
                        steering_angle = (steering_input + 1) * 90  # Scale to 0° to 180°
                        accelerator = inputs.get("pedals", {}).get("accelerator", 0)
                        brake = inputs.get("pedals", {}).get("brake", 0)
                        motor_speed = max(0, accelerator) - max(0, brake)

                        # Publish to ROS2 topics
                        self.steering_pub.publish(Float32(data=steering_angle))
                        self.motor_pub.publish(Float32(data=motor_speed))
                        self.accelerator_pub.publish(Float32(data=accelerator))
                        self.brake_pub.publish(Float32(data=brake))

                        self.get_logger().info(
                            f"Published: steering_angle={steering_angle}, motor_speed={motor_speed}, "
                            f"accelerator={accelerator}, brake={brake}"
                        )
                    except json.JSONDecodeError:
                        self.get_logger().error("Received invalid JSON message")
        except Exception as e:
            self.get_logger().error(f"Error with client {addr}: {e}")
        finally:
            self.get_logger().info(f"Connection with {addr} closed")

    def run(self):
        """Main server loop to accept and handle clients."""
        try:
            while rclpy.ok():
                conn, addr = self.server_socket.accept()
                self.get_logger().info(f"New client connected from {addr}")
                self.handle_client(conn, addr)
        except Exception as e:
            self.get_logger().error(f"Server error: {e}")
        finally:
            self.server_socket.close()

def main(args=None):
    rclpy.init(args=args)
    node = SocketROS2Bridge()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
