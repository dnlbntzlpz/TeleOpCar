import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import cv2

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        # Declare and retrieve parameters for camera device paths
        self.declare_parameter('front_left_camera', '/dev/video0')
        self.declare_parameter('front_right_camera', '/dev/video2')

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

        # Subscriber for obstacle detection data
        self.create_subscription(Float32, '/obstacle_distance_left', self.left_obstacle_callback, 10)
        self.create_subscription(Float32, '/obstacle_distance_right', self.right_obstacle_callback, 10)

        # Bridge to convert OpenCV images to ROS2 images
        self.bridge = CvBridge()

        # Obstacle data
        self.obstacle_distance_left = None
        self.obstacle_distance_left = None
        
        # Timer to periodically publish camera frames
        self.timer = self.create_timer(0.033, self.publish_frames)  # ~30 FPS

        self.get_logger().info("Camera Node Initialized")

    def left_obstacle_callback(self, msg):
        """Callback to receive obstacle detection data."""
        self.obstacle_distance_left = msg.data
    
    def right_obstacle_callback(self, msg):
        self.obstacle_distance_right = msg.data

    def overlay_obstacle_data(self, frame):
        """Overlay obstacle detection data on the video frame."""
        if self.obstacle_distance_left is not None and self.obstacle_distance_right is not None:
            label_left = f"Left Obstacle: {self.obstacle_distance_left:.2f}m"
            label_right = f"Right Obstacle: {self.obstacle_distance_right:.2f}m"
            # Draw label on the top-left corner
            cv2.putText(frame, label_left, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            cv2.putText(frame, label_right, (50, 80), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

    def publish_frames(self):
        # Capture and publish front-left camera frame
        ret_left, frame_left = self.front_left_cap.read()
        if ret_left:
            self.overlay_obstacle_data(frame_left)  # Add overlay
            ros_image_left = self.bridge.cv2_to_imgmsg(frame_left, encoding='bgr8')
            self.front_left_publisher.publish(ros_image_left)

        # Capture and publish front-right camera frame
        ret_right, frame_right = self.front_right_cap.read()
        if ret_right:
            self.overlay_obstacle_data(frame_right)  # Add overlay
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
