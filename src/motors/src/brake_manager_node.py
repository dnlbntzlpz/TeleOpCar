import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class BrakeManagerNode(Node):
    def __init__(self):
        super().__init__('brake_manager_node')

        # Subscribers for brake commands from different sources
        self.create_subscription(Float32, '/brake_command_ws', self.websocket_brake_callback, 10)
        self.create_subscription(Float32, '/brake_command_obs', self.obstacle_brake_callback, 10)

        # Publisher for the final brake command
        self.final_brake_publisher = self.create_publisher(Float32, '/final_brake_command', 10)

        # Internal states for brake values
        self.websocket_brake = 0.0  # Brake value from WebSocket controller
        self.obstacle_brake = 0.0  # Brake value from obstacle detection

        self.get_logger().info("Brake Manager Node Initialized")

    def websocket_brake_callback(self, msg):
        """Callback for brake command from WebSocket controller."""
        self.websocket_brake = msg.data
        self.update_final_brake()

    def obstacle_brake_callback(self, msg):
        """Callback for brake command from obstacle detection."""
        self.obstacle_brake = msg.data
        self.update_final_brake()

    def update_final_brake(self):
        """Publish the final brake command based on inputs."""
        final_brake = min(self.websocket_brake, self.obstacle_brake)
        self.final_brake_publisher.publish(Float32(data=final_brake))
        self.get_logger().info(f"Final Brake Command: {final_brake}")


def main(args=None):
    rclpy.init(args=args)
    node = BrakeManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
