import rclpy
from rclpy.node import Node
from gpiozero import PWMOutputDevice, OutputDevice


class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')

        # Declare and retrieve parameters
        self.declare_parameter('motor_in1_pin', 12)  # GPIO pin for IN1
        self.declare_parameter('motor_in2_pin', 13)  # GPIO pin for IN2

        in1_pin = self.get_parameter('motor_in1_pin').value
        in2_pin = self.get_parameter('motor_in2_pin').value

        # Setup GPIO pins
        self.in1 = PWMOutputDevice(in1_pin)
        self.in2 = OutputDevice(in2_pin)

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
            command = msg.data.split()  # Expecting format: "speed direction"
            speed = float(command[0])  # Speed value (0.0 to 1.0)
            direction = command[1].lower()  # Direction: "forward" or "reverse"

            # Control motor based on direction
            if direction == 'forward':
                self.in1.value = speed
                self.in2.off()
            elif direction == 'reverse':
                self.in1.value = 0  # Stop IN1
                self.in2.on()  # Set IN2 for reverse
            else:
                self.in1.off()
                self.in2.off()

            self.get_logger().info(f"Motor set to {direction} at speed {speed}")
        except Exception as e:
            self.get_logger().error(f"Invalid command: {msg.data} | Error: {str(e)}")

    def destroy_node(self):
        # Cleanup GPIO pins
        self.in1.close()
        self.in2.close()
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
