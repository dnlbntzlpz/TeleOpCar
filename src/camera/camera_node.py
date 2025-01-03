import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        # Declare and retrieve parameters for camera device paths
        self.declare_parameter('front_left_camera', '/dev/video0')
        self.declare_parameter('front_right_camera', '/dev/video1')

        self.front_left_camera_path = self.get_parameter('front_left_camera').value
        self.front_right_camera_path = self.get_parameter('front_right_camera').value

        # Initialize OpenCV video capture for both cameras
        self.front_left_cap = cv2.VideoCapture(self.front_left_camera_path)
        self.front_right_cap = cv2.VideoCapture(self.front_right_camera_path)

        # Set resolution (optional)
        self.front_left_cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.front_left_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.front_right_cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.front_right_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        # Publishers for camera feeds
        self.front_left_publisher = self.create_publisher(Image, '/camera/front_left/image_raw', 10)
        self.front_right_publisher = self.create_publisher(Image, '/camera/front_right/image_raw', 10)

        # Bridge to convert OpenCV images to ROS2 images
        self.bridge = CvBridge()

        # Timer to periodically publish camera frames
        self.timer = self.create_timer(0.033, self.publish_frames)  # ~30 FPS

        self.get_logger().info("Camera Node Initialized")

    def publish_frames(self):
        # Capture and publish front-left camera frame
        ret_left, frame_left = self.front_left_cap.read()
        if ret_left:
            ros_image_left = self.bridge.cv2_to_imgmsg(frame_left, encoding='bgr8')
            self.front_left_publisher.publish(ros_image_left)

        # Capture and publish front-right camera frame
        ret_right, frame_right = self.front_right_cap.read()
        if ret_right:
            ros_image_right = self.bridge.cv2_to_imgmsg(frame_right, encoding='bgr8')
            self.front_right_publisher.publish(ros_image_right)

    def destroy_node(self):
        # Release camera resources
        self.front_left_cap.release()
        self.front_right_cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
