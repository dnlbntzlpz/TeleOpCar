import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import time

class BrakeManagerNode(Node):
    def __init__(self):
        super().__init__('brake_manager_node')

        # Subscribers for brake commands from different sources
        self.create_subscription(Float32, '/brake_command_ws', self.websocket_brake_callback, 1)
        self.create_subscription(Float32, '/brake_command_obs', self.obstacle_brake_callback, 1)

        # Publisher for the final brake command
        self.final_brake_publisher = self.create_publisher(Float32, '/final_brake_command', 1)

        # Internal states for brake values
        self.websocket_brake = 0.0  # Brake value from WebSocket controller
        self.obstacle_brake = 0.0  # Brake value from obstacle detection
        self.last_final_brake = None  # Store the last published brake value
        self.last_publish_time = time.time()  # Track the last publish time
        self.last_update_time = time.time()  # Track the last update time

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
        final_brake = max(self.websocket_brake, self.obstacle_brake)

        # Throttle publish rate to every 0.5 seconds
        current_time = time.time()
        if (self.last_final_brake != final_brake) or (current_time - self.last_publish_time > 0.5):
            # Debounce logic: only update if enough time has passed since the last update
            if current_time - self.last_update_time > 0.1:  # 100 ms debounce
                self.final_brake_publisher.publish(Float32(data=final_brake))
                self.get_logger().info(f"Final Brake Command: {final_brake}")
                self.last_final_brake = final_brake
                self.last_publish_time = current_time
                self.last_update_time = current_time  # Update last update time

        # Log incoming values for debugging only if they change
        if self.websocket_brake != self.last_final_brake or self.obstacle_brake != self.last_final_brake:
            self.get_logger().debug(f"WebSocket Brake: {self.websocket_brake}, Obstacle Brake: {self.obstacle_brake}")

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
