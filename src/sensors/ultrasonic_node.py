import rclpy
from rclpy.node import Node
from gpiozero import DistanceSensor
from std_msgs.msg import Float32

class UltrasonicNode(Node):
    def __init__(self):
        super().__init__('ultrasonic_node')

        # Declare and retrieve GPIO pin parameters
        self.declare_parameter('left_trigger_pin', 17)
        self.declare_parameter('left_echo_pin', 27)
        self.declare_parameter('right_trigger_pin', 22)
        self.declare_parameter('right_echo_pin', 24)

        left_trigger = self.get_parameter('left_trigger_pin').value
        left_echo = self.get_parameter('left_echo_pin').value
        right_trigger = self.get_parameter('right_trigger_pin').value
        right_echo = self.get_parameter('right_echo_pin').value

        # Initialize sensors
        self.left_sensor = DistanceSensor(echo=left_echo, trigger=left_trigger, max_distance=2.0)
        self.right_sensor = DistanceSensor(echo=right_echo, trigger=right_trigger, max_distance=2.0)

        # Publishers for distances
        self.left_publisher = self.create_publisher(Float32, '/ultrasonic/left', 10)
        self.right_publisher = self.create_publisher(Float32, '/ultrasonic/right', 10)

        # Timer to periodically read distances
        self.timer = self.create_timer(0.1, self.publish_distances)  # 10 Hz

        self.get_logger().info("Ultrasonic Node Initialized")

    def publish_distances(self):
        try:
            # Read distances
            left_distance = self.left_sensor.distance  # In meters
            right_distance = self.right_sensor.distance  # In meters

            # Publish distances
            self.left_publisher.publish(Float32(data=left_distance))
            self.right_publisher.publish(Float32(data=right_distance))

            self.get_logger().info(f"Left: {left_distance:.2f}m, Right: {right_distance:.2f}m")
        except Exception as e:
            self.get_logger().error(f"Error reading sensors: {str(e)}")

    def destroy_node(self):
        # Cleanup resources
        self.left_sensor.close()
        self.right_sensor.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
