import rclpy
from rclpy.node import Node
from gpiozero import DistanceSensor
from std_msgs.msg import Float32

class ObstacleDetectionNode(Node):
    def __init__(self):
        super().__init__('obstacle_detection_node')

        # Declare and retrieve GPIO pin parameters
        self.declare_parameter('left_trigger_pin', 17)
        self.declare_parameter('left_echo_pin', 27)
        self.declare_parameter('right_trigger_pin', 22)
        self.declare_parameter('right_echo_pin', 24)

        left_trigger = self.get_parameter('left_trigger_pin').value
        left_echo = self.get_parameter('left_echo_pin').value
        right_trigger = self.get_parameter('right_trigger_pin').value
        right_echo = self.get_parameter('right_echo_pin').value

        # Declare detection threshold parameters
        self.declare_parameter('obstacle_threshold', 0.3)  # 30 cm
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').value

        # Initialize sensors
        self.left_sensor = DistanceSensor(echo=left_echo, trigger=left_trigger, max_distance=2.0)
        self.right_sensor = DistanceSensor(echo=right_echo, trigger=right_trigger, max_distance=2.0)

        # Publishers for control commands
        self.brake_publisher = self.create_publisher(Float32, '/brake_command_obs', 10)
        self.left_distance_publisher = self.create_publisher(Float32, '/obstacle_distance_left', 10)
        self.right_distance_publisher = self.create_publisher(Float32, '/obstacle_distance_right', 10)

        # Timer to periodically read distances
        self.timer = self.create_timer(0.1, self.check_obstacles)  # 10 Hz

        self.get_logger().info("Obstacle Detection Node Initialized")

    def check_obstacles(self):
        try:
            # Read distances
            left_distance = self.left_sensor.distance  # In meters
            right_distance = self.right_sensor.distance  # In meters

            # Initialize commands
            brake_value = 0.0  # No brake by default

            # Obstacle detection logic (maybe it should be or instead of and)
            if left_distance < self.obstacle_threshold or right_distance < self.obstacle_threshold:
                brake_value = 0.0  # Full brake
            else:
                brake_value = 1.0

            #Publish distances
            self.left_distance_publisher.publish(Float32(data=left_distance))
            self.right_distance_publisher.publish(Float32(data=right_distance))

            # Publish commands
            self.brake_publisher.publish(Float32(data=brake_value))

            # Log actions
            self.get_logger().info(
                f"Distances - Left: {left_distance:.2f}m, Right: {right_distance:.2f}m | "
                f"Brake: {brake_value}"
            )
        except Exception as e:
            self.get_logger().error(f"Error reading sensors: {str(e)}")

    def destroy_node(self):
        # Cleanup resources
        self.left_sensor.close()
        self.right_sensor.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
