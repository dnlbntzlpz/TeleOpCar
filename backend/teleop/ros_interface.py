import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class ROS2Interface(Node):
    def __init__(self):
        # Initialize rclpy if it hasn't been initialized
        if not rclpy.ok():
            rclpy.init()
        
        super().__init__('ros2_interface')

        # Publishers
        self.accelerator_publisher = self.create_publisher(Float32, '/accelerator_command', 10)
        self.steering_publisher = self.create_publisher(Float32, '/servo_command', 10)
        self.brake_publisher = self.create_publisher(Float32, '/brake_command_ws', 10)

    def publish_accelerator(self, value):
        self.accelerator_publisher.publish(Float32(data=value))
        self.get_logger().info(f"Published accelerator: {value}")

    def publish_steering(self, angle):
        self.steering_publisher.publish(Float32(data=angle))
        self.get_logger().info(f"Published steering angle: {angle}")

    def publish_brake(self, value):
        self.brake_publisher.publish(Float32(data=value))
        self.get_logger().info(f"Published brake: {value}")
