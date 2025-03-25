import rclpy
from rclpy.node import Node
import lgpio
import time
import math
from std_msgs.msg import Float32
from collections import deque
import threading
from concurrent.futures import ThreadPoolExecutor  # Import ThreadPoolExecutor

class ObstacleDetectionNode(Node):
    SPEED_OF_SOUND_HALF = 34300 / 2 / 100.0  # Pre-compute (m/s), conversion factor

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
        self.declare_parameter('obstacle_threshold', 0.5)  # 1 meter
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').value

        # Initialize GPIO
        self.chip = lgpio.gpiochip_open(0)
        lgpio.gpio_claim_output(self.chip, self.left_trigger)
        lgpio.gpio_claim_input(self.chip, self.left_echo)
        lgpio.gpio_claim_output(self.chip, self.right_trigger)
        lgpio.gpio_claim_input(self.chip, self.right_echo)

        # Validate GPIO setup
        self.validate_gpio(self.left_echo, "left echo")
        self.validate_gpio(self.right_echo, "right echo")

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

        # Add a lock for thread-safe GPIO operations
        self.gpio_lock = threading.Lock()

        # Add history for distance readings using deque for better performance
        initial_distance = 4.0  # or any default "safe" starting value
        self.left_distance_history = deque([initial_distance] * 5, maxlen=5)
        self.right_distance_history = deque([initial_distance] * 5, maxlen=5)
        self.left_distance_sum = initial_distance * 5
        self.right_distance_sum = initial_distance * 5

        # Initialize ThreadPoolExecutor
        self.thread_pool = ThreadPoolExecutor(max_workers=2)  # Renamed to avoid conflict

        self.get_logger().info("Obstacle Detection Node Initialized")

    def validate_gpio(self, pin, name):
        """Validate GPIO setup by checking if the pin can be read."""
        try:
            value = lgpio.gpio_read(self.chip, pin)
            if value not in (0, 1):
                self.get_logger().error(f"Failed to read {name} GPIO. Check wiring.")
        except Exception as e:
            self.get_logger().error(f"Error reading {name} GPIO: {str(e)}")

    def motor_speed_callback(self, msg):
        """Callback to update the current motor speed."""
        self.motor_speed = msg.data

    def get_distance(self, trigger, echo):
        MAX_TIMEOUT = 0.1

        # Use a lock to ensure thread-safe access to GPIO
        with self.gpio_lock:
            lgpio.gpio_write(self.chip, trigger, 1)
            time.sleep(0.00001)
            lgpio.gpio_write(self.chip, trigger, 0)

            timeout_start = time.monotonic()
            start_time = None

            while lgpio.gpio_read(self.chip, echo) == 0:
                if time.monotonic() - timeout_start > MAX_TIMEOUT:
                    return float('inf')
                start_time = time.monotonic()

            timeout_start = time.monotonic()
            stop_time = None

            while lgpio.gpio_read(self.chip, echo) == 1:
                if time.monotonic() - timeout_start > MAX_TIMEOUT:
                    return float('inf')
                stop_time = time.monotonic()

        # Safety check
        if start_time is None or stop_time is None:
            return float('inf')

        elapsed_time = stop_time - start_time
        distance = elapsed_time * self.SPEED_OF_SOUND_HALF

        self.get_logger().debug(f"Distance: {distance}m (elapsed: {elapsed_time}s)")

        return min(distance, 4.0)

    def is_obstacle_detected(self, current_distance):
        """Check if an obstacle is detected immediately"""
        return current_distance < self.obstacle_threshold

    def get_smoothed_distance(self, distance_history, distance_sum, new_distance):
        """Update the distance history and return the smoothed distance more efficiently."""
        # Ignore invalid readings
        if math.isinf(new_distance):
            return distance_sum / len(distance_history), distance_sum

        # Remove the oldest value from the sum
        old_value = distance_history[0]
        updated_sum = distance_sum - old_value + new_distance
        
        # Update the history (deque will handle removal automatically)
        distance_history.append(new_distance)
        
        # Return the average
        return updated_sum / len(distance_history), updated_sum

    def check_obstacles(self):
        try:
            # Use ThreadPoolExecutor to read sensors in parallel
            left_future = self.thread_pool.submit(self.get_distance, self.left_trigger, self.left_echo)
            right_future = self.thread_pool.submit(self.get_distance, self.right_trigger, self.right_echo)

            left_distance = left_future.result()
            right_distance = right_future.result()

            # Smooth the distances and update the sums
            smoothed_left_distance, self.left_distance_sum = self.get_smoothed_distance(
                self.left_distance_history, 
                self.left_distance_sum, 
                left_distance
            )
            
            smoothed_right_distance, self.right_distance_sum = self.get_smoothed_distance(
                self.right_distance_history,
                self.right_distance_sum,
                right_distance
            )
            
            # Check for NaN values and replace with a safe value
            if math.isnan(smoothed_left_distance):
                smoothed_left_distance = float('inf')
                self.get_logger().warn("NaN detected for left sensor, replacing with infinity")
                
            if math.isnan(smoothed_right_distance):
                smoothed_right_distance = float('inf')
                self.get_logger().warn("NaN detected for right sensor, replacing with infinity")
            
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
        self.thread_pool.shutdown(wait=True)  # Ensure all threads are finished
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