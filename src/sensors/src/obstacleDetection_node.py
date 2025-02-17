import rclpy
from rclpy.node import Node
import lgpio
import time
from std_msgs.msg import Float32

class ObstacleDetectionNode(Node):
    def __init__(self):
        super().__init__('obstacle_detection_node')

        # Declare and retrieve GPIO pin parameters
        self.declare_parameter('left_trigger_pin', 17)
        self.declare_parameter('left_echo_pin', 27)
        self.declare_parameter('right_trigger_pin', 22)
        self.declare_parameter('right_echo_pin', 24)

        self.left_trigger = self.get_parameter('left_trigger_pin').value
        self.left_echo = self.get_parameter('left_echo_pin').value
        self.right_trigger = self.get_parameter('right_trigger_pin').value
        self.right_echo = self.get_parameter('right_echo_pin').value

        # Declare detection threshold parameters
        self.declare_parameter('obstacle_threshold', 0.3)  # 30 cm
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').value

        # Initialize GPIO
        self.chip = lgpio.gpiochip_open(0)
        lgpio.gpio_claim_output(self.chip, self.left_trigger)
        lgpio.gpio_claim_input(self.chip, self.left_echo)
        lgpio.gpio_claim_output(self.chip, self.right_trigger)
        lgpio.gpio_claim_input(self.chip, self.right_echo)

        # Publishers for control commands
        self.brake_publisher = self.create_publisher(Float32, '/brake_command_obs', 10)
        self.left_distance_publisher = self.create_publisher(Float32, '/obstacle_distance_left', 10)
        self.right_distance_publisher = self.create_publisher(Float32, '/obstacle_distance_right', 10)
        self.steering_publisher = self.create_publisher(Float32, '/steering_command_obs', 10)

        # Timer to periodically read distances
        self.timer = self.create_timer(0.5, self.check_obstacles)  # 10 Hz

        self.get_logger().info("Obstacle Detection Node Initialized")

    def get_distance(self, trigger, echo):
        """Measures distance using an ultrasonic sensor"""
        lgpio.gpio_write(self.chip, trigger, 1)
        time.sleep(0.00001)
        lgpio.gpio_write(self.chip, trigger, 0)

        start_time = time.time()
        stop_time = time.time()

        while lgpio.gpio_read(self.chip, echo) == 0:
            start_time = time.time()

        while lgpio.gpio_read(self.chip, echo) == 1:
            stop_time = time.time()

        elapsed_time = stop_time - start_time
        distance = (elapsed_time * 34300) / 2  # Convert to cm
        return distance / 100.0  # Convert to meters

    def check_obstacles(self):
        try:
            # Read distances
            left_distance = self.get_distance(self.left_trigger, self.left_echo)
            right_distance = self.get_distance(self.right_trigger, self.right_echo)

            # Initialize commands Obstacle detection logic
            brake_value = 0.0  # Default: no brake
            steering_value = 0.0  # Centered steering

            # Obstacle detection logic (maybe it should be or instead of and)
            if left_distance < self.obstacle_threshold and right_distance < self.obstacle_threshold:
                brake_value = 0.0  # Full brake
            elif left_distance < self.obstacle_threshold:
                steering_value = 0.75  # Steer slightly right
            elif right_distance < self.obstacle_threshold:
                steering_value = -0.75  # Steer slightly left
            else:
                brake_value = 1.0

            # Publish distances
            self.left_distance_publisher.publish(Float32(data=left_distance))
            self.right_distance_publisher.publish(Float32(data=right_distance))

            # Publish brake command
            self.brake_publisher.publish(Float32(data=brake_value))

            # Publish steering command
            self.steering_publisher.publish(Float32(data=steering_value))

            # Log actions
            self.get_logger().info(
                f"Distances - Left: {left_distance:.2f}m, Right: {right_distance:.2f}m | "
                f"Brake: {brake_value}, Steering: {steering_value}"
            )
        except Exception as e:
            self.get_logger().error(f"Error reading sensors: {str(e)}")

    def destroy_node(self):
        # Cleanup resources
        lgpio.gpiochip_close(self.chip)
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