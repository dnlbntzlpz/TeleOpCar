import rclpy
from rclpy.node import Node
from gpiozero import PWMOutputDevice, OutputDevice
from std_msgs.msg import String  # Import the String message type

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

        # Subscriber to control motor speed and direction
        self.subscription = self.create_subscription(
            String,
            'motor_command',
            self.motor_command_callback,
            10
        )

        self.get_logger().info("Motor Control Node Initialized")

    def motor_command_callback(self, msg):
        try:
            command = msg.data.split()  # Expecting format: "left_speed right_speed left_direction right_direction"
            left_speed = float(command[0])  # Speed for Left Motor (0.0 to 1.0)
            right_speed = float(command[1])  # Speed for Right Motor (0.0 to 1.0)
            left_direction = command[2].lower()  # Direction for Left Motor: "forward" or "reverse"
            right_direction = command[3].lower()  # Direction for Right Motor: "forward" or "reverse"

            # Control Left Motor
            if left_direction == 'forward':
                self.left_pwm.value = left_speed
                self.left_dir.off()
            elif left_direction == 'reverse':
                self.left_pwm.value = left_speed
                self.left_dir.on()
            else:
                self.left_pwm.off()
                self.left_dir.off()

            # Control Right Motor
            if right_direction == 'forward':
                self.right_pwm.value = right_speed
                self.right_dir.off()
            elif right_direction == 'reverse':
                self.right_pwm.value = right_speed
                self.right_dir.on()
            else:
                self.right_pwm.off()
                self.right_dir.off()

            self.get_logger().info(f"Left Motor: {left_direction} at speed {left_speed}, Right Motor: {right_direction} at speed {right_speed}")
        except Exception as e:
            self.get_logger().error(f"Invalid command: {msg.data} | Error: {str(e)}")

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
