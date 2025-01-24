import rclpy
from rclpy.node import Node
from gpiozero import PWMOutputDevice
from std_msgs.msg import Float32

class ServoControlNode(Node):
    def __init__(self):
        super().__init__('servo_control_node')

        # Declare and retrieve parameters
        self.declare_parameter('servo_pwm_pin', 12)  # GPIO pin for the servo
        pwm_pin = self.get_parameter('servo_pwm_pin').value

        # Initialize PWM output device for the servo
        self.servo = PWMOutputDevice(pwm_pin, frequency=50)  # 50Hz standard for servos

        # Initialize current steering angle and override state
        self.current_angle = 90.0 # Start centered at 90
        self.override_active = False # Flag for obstacle avoidance override
        self.last_obstavle_angle = 90.0 # Last angle set by obstacle avoidance

        # Subscriber to control the servo angle
        self.create_subscription(Float32, 'servo_command', self.servo_command_callback, 10)
        self.create_subscription(Float32, '/steering_command_obs', self.obstacle_command_callback, 10)

        self.get_logger().info("Servo Control Node Initialized")

    def servo_command_callback(self, msg):
        if not self.override_active:
            angle = msg.data  # Expected angle in degrees (0-180)
            self.set_servo_angle(angle)
        else:
            self.get_logger().info("Override active, ignoring controller command")

    def obstacle_command_callback(self, msg):
        """Callback for steering commands from ovstacle avoidance"""
        steering_value = msg.data #STeering value from obstacle avoidance (-1.0 to 1.0)

        if steering_value != 0.0: #Obstacle detected
            self.override_active = True
            angle = 90.0 + (steering_value * 45.0) # -1 -> 45, 0 -> 90, 1 -> 135
            self.set_servo_angle(angle)
            self.get_logger().info(f"Obstacle detected, overriding to angle: {angle}")
        else:
            #No obstacle detected, deactivate override
            if self.override_active:
                self.get_logger().info("No Obstacle detected, returning control to the contoller.")
            self.override_active = False
        
        
    def steering_command_callback(self, msg):
        """Callback for assistive steering commands."""
        steering_value = msg.data  # Steering value from obstacle detection (-1.0 to 1.0)

        # Map steering values (-1.0, 0.0, 1.0) to servo angles (0°, 90°, 180°)
        if steering_value < -1.0 or steering_value > 1.0:
            self.get_logger().warn(f"Invalid steering value: {steering_value}. Must be between -1.0 and 1.0.")
            return

        # Map the steering value to an angle
        angle = 90.0 + (steering_value * 45.0)  # -1 -> 45°, 0 -> 90°, 1 -> 135°
        self.set_servo_angle(angle)

    def set_servo_angle(self, angle):
        """
        Moves the servo to a specific angle.
        Maps 0-180 degrees to 2.5%-12.5% duty cycle.
        """
        if angle < 0.0 or angle > 180.0:
            self.get_logger().warn(f"Invalid angle: {angle}. Must be between 0 and 180 degrees.")
            return

        # Map angle to duty cycle (500-2500µs pulse width -> 2.5%-12.5% duty cycle)
        pulse_width_us = 500 + (angle / 180.0) * 2000  # Map angle to 500-2500µs range
        duty_cycle = pulse_width_us / 20000  # Convert pulse width to duty cycle (20ms period)

        self.servo.value = duty_cycle
        self.current_angle = angle
        self.get_logger().info(f"Set servo to {angle}° (Duty Cycle: {duty_cycle:.4f})")

    def destroy_node(self):
        # Cleanup GPIO
        self.servo.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ServoControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
