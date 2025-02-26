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
        self.declare_parameter('obstacle_threshold', 1.0)  # 1 meter
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').value

        # Initialize GPIO
        self.chip = lgpio.gpiochip_open(0)
        lgpio.gpio_claim_output(self.chip, self.left_trigger)
        lgpio.gpio_claim_input(self.chip, self.left_echo)
        lgpio.gpio_claim_output(self.chip, self.right_trigger)
        lgpio.gpio_claim_input(self.chip, self.right_echo)

        # Publishers for control commands with smaller queue size
        self.brake_publisher = self.create_publisher(Float32, '/brake_command_obs', 1)
        self.left_distance_publisher = self.create_publisher(Float32, '/obstacle_distance_left', 1)
        self.right_distance_publisher = self.create_publisher(Float32, '/obstacle_distance_right', 1)
        self.steering_publisher = self.create_publisher(Float32, '/steering_command_obs', 1)

        # Subscribe to motor speed command to determine direction
        self.motor_speed = 0.0
        self.create_subscription(Float32, '/accelerator_command', self.motor_speed_callback, 1)

        # Timer to periodically read distances with lower frequency
        self.timer = self.create_timer(0.2, self.check_obstacles)  # 5 Hz

        # Add history for distance readings
        self.left_distance_history = [float('inf')] * 5  # Increase history size for smoothing
        self.right_distance_history = [float('inf')] * 5  # Increase history size for smoothing

        self.get_logger().info("Obstacle Detection Node Initialized")

    def motor_speed_callback(self, msg):
        """Callback to update the current motor speed."""
        self.motor_speed = msg.data

    def get_distance(self, trigger, echo):
        """Measures distance using an ultrasonic sensor with timeout"""
        MAX_TIMEOUT = 0.1  # 100ms timeout
        
        lgpio.gpio_write(self.chip, trigger, 1)
        time.sleep(0.00001)
        lgpio.gpio_write(self.chip, trigger, 0)

        start_time = time.time()
        timeout_start = start_time

        # Wait for echo to go high with timeout
        while lgpio.gpio_read(self.chip, echo) == 0:
            start_time = time.time()
            if time.time() - timeout_start > MAX_TIMEOUT:
                return float('inf')  # Return infinite distance on timeout

        timeout_start = time.time()
        # Wait for echo to go low with timeout
        while lgpio.gpio_read(self.chip, echo) == 1:
            if time.time() - timeout_start > MAX_TIMEOUT:
                return float('inf')  # Return infinite distance on timeout
            stop_time = time.time()

        elapsed_time = stop_time - start_time
        distance = (elapsed_time * 34300) / 2  # Convert to cm
        return min(distance / 100.0, 4.0)  # Convert to meters, cap at 4 meters

    def is_obstacle_detected(self, current_distance):
        """Check if an obstacle is detected immediately"""
        return current_distance < self.obstacle_threshold

    def get_smoothed_distance(self, distance_history, new_distance):
        """Update the distance history and return the smoothed distance."""
        distance_history.pop(0)  # Remove the oldest reading
        distance_history.append(new_distance)  # Add the new reading
        return sum(distance_history) / len(distance_history)  # Return the average

    def check_obstacles(self):
        try:
            # Read distances with small delay between readings
            left_distance = self.get_distance(self.left_trigger, self.left_echo)
            time.sleep(0.01)  # Small delay between readings
            right_distance = self.get_distance(self.right_trigger, self.right_echo)

            # Smooth the distances
            smoothed_left_distance = self.get_smoothed_distance(self.left_distance_history, left_distance)
            smoothed_right_distance = self.get_smoothed_distance(self.right_distance_history, right_distance)

            # Initialize commands
            brake_value = 0.0  # Default: no brake
            steering_value = 0.0  # Centered steering

            # Check for obstacles immediately using smoothed distances
            left_obstacle = self.is_obstacle_detected(smoothed_left_distance)
            right_obstacle = self.is_obstacle_detected(smoothed_right_distance)

            # Obstacle detection logic
            if left_obstacle and right_obstacle:
                if self.motor_speed > 0:  # Only apply brake when moving forward
                    brake_value = 1.0  # Full brake
            elif left_obstacle:
                steering_value = 0.75  # Steer slightly right
            elif right_obstacle:
                steering_value = -0.75  # Steer slightly left

            # Publish distances (use smoothed readings)
            self.left_distance_publisher.publish(Float32(data=smoothed_left_distance))
            self.right_distance_publisher.publish(Float32(data=smoothed_right_distance))

            # Publish brake command
            self.brake_publisher.publish(Float32(data=brake_value))

            # Publish steering command
            self.steering_publisher.publish(Float32(data=steering_value))

            # Log actions
            self.get_logger().info(
                f"Smoothed Distances - Left: {smoothed_left_distance:.2f}m, Right: {smoothed_right_distance:.2f}m | "
                f"Obstacles - Left: {left_obstacle}, Right: {right_obstacle} | "
                f"Brake: {brake_value}, Steering: {steering_value}, Motor Speed: {self.motor_speed}"
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