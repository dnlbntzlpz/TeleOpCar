import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import Bool

class ROS2Interface(Node):
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        
        super().__init__('ros2_interface')

        # Publishers
        self.accelerator_publisher = self.create_publisher(Float32, '/accelerator_command', 1)
        self.direction_publisher = self.create_publisher(Bool, '/direction_command', 1)
        self.steering_publisher = self.create_publisher(Float32, '/servo_command', 1)
        self.brake_publisher = self.create_publisher(Float32, '/brake_command_ws', 1)

    def publish_accelerator(self, value):
        self.accelerator_publisher.publish(Float32(data=value))
        self.get_logger().info(f"Published accelerator: {value}")

    # def publish_accelerator(self, value):
    #     # Determine direction
    #     direction = Bool(data=(value >= 0))  # True for forward, False for reverse
    #     self.direction_publisher.publish(direction)

    #     # Publish absolute value of acceleration
    #     self.accelerator_publisher.publish(Float32(data=abs(value)))
    #     self.get_logger().info(f"Published accelerator: {abs(value)}, direction: {'forward' if direction.data else 'reverse'}")

    def publish_steering(self, angle):
        self.steering_publisher.publish(Float32(data=angle))
        self.get_logger().info(f"Published steering angle: {angle}")

    def publish_brake(self, value):
        self.brake_publisher.publish(Float32(data=value))
        self.get_logger().info(f"Published brake: {value}")
