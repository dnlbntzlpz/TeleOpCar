import socket
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class SocketROS2Bridge(Node):
    def __init__(self):
        super().__init__('socket_ros2_bridge')

        # ROS2 publishers
        self.steering_pub = self.create_publisher(Float32, '/servo_command', 10)
        self.motor_pub = self.create_publisher(Float32, '/motor_command', 10)

        # Server configuration
        self.HOST = "0.0.0.0"  # Listen on all network interfaces
        self.PORT = 8765       # Port for the TCP server
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind((self.HOST, self.PORT))
        self.server_socket.listen(1)  # Allow 1 client to connect

        self.get_logger().info(f"Server started on {self.HOST}:{self.PORT}, waiting for a connection...")

    def handle_connection(self):
        try:
            conn, addr = self.server_socket.accept()  # Accept incoming connection
            self.get_logger().info(f"Connected by {addr}")
            with conn:
                while rclpy.ok():
                    data = conn.recv(1024)  # Receive up to 1024 bytes
                    if not data:
                        break
                    message = data.decode()
                    self.get_logger().info(f"Received: {message}")

                    try:
                        # Parse JSON data
                        inputs = json.loads(message)

                        # Extract steering angle and motor speed
                        steering_angle = inputs.get("pedals", {}).get("wheel", 0) * 90  # Scale to -90° to 90°
                        accelerator = inputs.get("pedals", {}).get("accelerator", 0)
                        brake = inputs.get("pedals", {}).get("brake", 0)
                        motor_speed = max(0, accelerator) - max(0, brake)

                        # Publish to ROS2 topics
                        self.steering_pub.publish(Float32(data=steering_angle))
                        self.motor_pub.publish(Float32(data=motor_speed))

                        self.get_logger().info(f"Published: steering_angle={steering_angle}, motor_speed={motor_speed}")
                    except json.JSONDecodeError:
                        self.get_logger().error("Received invalid JSON message")
        except Exception as e:
            self.get_logger().error(f"Server error: {e}")
        finally:
            self.get_logger().info("Connection closed")
            self.server_socket.close()

    def run(self):
        self.handle_connection()

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
