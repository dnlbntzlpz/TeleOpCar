import rclpy
from rclpy.node import Node
from gpiozero import PWMOutputDevice, OutputDevice
from std_msgs.msg import Float32, Bool

class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')

        # Declare and retrieve parameters for Left Motor
        self.declare_parameter('left_motor_pwm_pin', 13)  # GPIO pin for Left Motor PWM
        self.declare_parameter('left_motor_dir_pin', 16)  # GPIO pin for Left Motor Direction

        # Declare and retrieve parameters for Right Motor
        self.declare_parameter('right_motor_pwm_pin', 18)  # GPIO pin for Right Motor PWM
        self.declare_parameter('right_motor_dir_pin', 23)  # GPIO pin for Right Motor Direction

        left_pwm_pin = self.get_parameter('left_motor_pwm_pin').value
        left_dir_pin = self.get_parameter('left_motor_dir_pin').value
        right_pwm_pin = self.get_parameter('right_motor_pwm_pin').value
        right_dir_pin = self.get_parameter('right_motor_dir_pin').value

        # Setup GPIO pins for Left Motor
        self.left_pwm = PWMOutputDevice(left_pwm_pin)
        self.left_dir = OutputDevice(left_dir_pin)

        # Setup GPIO pins for Right Motor
        self.right_pwm = PWMOutputDevice(right_pwm_pin)
        self.right_dir = OutputDevice(right_dir_pin)

        # Internal states
        self.accelerator = 0.0  # Accelerator value
        self.brake = 0.0        # Brake value
        self.direction = True  # Direction of the motor

        # Subscribers for accelerator and brake
        self.create_subscription(Float32, '/accelerator_command', self.accelerator_callback, 1)
        self.create_subscription(Float32, '/final_brake_command', self.brake_callback, 1)
        self.create_subscription(Bool, '/direction_command', self.direction_callback, 1)

        self.get_logger().info("Motor Control Node Initialized")

    def accelerator_callback(self, msg):
        """Callback for accelerator command."""
        self.accelerator = msg.data
        self.update_motor_speed()

    def brake_callback(self, msg):
        """Callback for brake command."""
        self.brake = msg.data
        self.update_motor_speed()

    def direction_callback(self, msg):
        """Callback for direction command."""
        self.direction = msg.data
        self.update_motor_speed()

    def update_motor_speed(self):
        """Updates motor speed and direction based on accelerator and brake."""
        if self.brake >= 0.5:
            # Full brake, stop motors
            self.left_pwm.value = 0.0
            self.right_pwm.value = 0.0
            self.get_logger().info("Brake activated: Motors stopped.")

        else:
            speed = abs(self.accelerator)  # Get absolute speed value
            is_forward = self.direction  # Determine direction
            print(f"Direction: {'forward' if is_forward else 'reverse'}")
            
            # Set motor speed
            self.left_pwm.value = speed
            self.right_pwm.value = speed
 
            # Set motor direction
            if is_forward:
                self.left_dir.off()  # Forward
                self.right_dir.off()
                direction = "forward"
            else:
                self.left_dir.on()  # Reverse
                self.right_dir.on()
                direction = "reverse"

            self.get_logger().info(f"Motors set to {direction} at speed: {speed}")

    def destroy_node(self):
        # Cleanup GPIO pins
        self.left_pwm.close()
        self.left_dir.close()
        self.right_pwm.close()
        self.right_dir.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
