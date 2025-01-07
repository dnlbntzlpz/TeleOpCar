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

        # Subscriber to control the servo angle
        self.subscription = self.create_subscription(
            Float32,
            'servo_command',
            self.servo_command_callback,
            10
        )

        self.get_logger().info("Servo Control Node Initialized")

    def servo_command_callback(self, msg):
        angle = msg.data  # Expected angle in degrees (0-180)
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
