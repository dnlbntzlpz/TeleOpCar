import threading
import rclpy
from django.apps import AppConfig
from .ros_interface import ROS2Interface

class TeleopConfig(AppConfig):
    name = 'teleop'

    def ready(self):
        # Start the ROS2 node in a separate thread
        threading.Thread(target=self.start_ros2_node, daemon=True).start()

    def start_ros2_node(self):
        rclpy.init()
        node = ROS2Interface()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()
