from django.shortcuts import render
from django.http import JsonResponse
from .ros_interface import ROS2Interface
import rclpy

# Ensure rclpy is initialized only once
if not rclpy.ok():
    rclpy.init()

ros2_interface = ROS2Interface()

def index(request):
    return render(request, 'teleop/index.html')

def send_command(request):
    command = request.GET.get('command', '')

    if command == "forward":
        ros2_interface.publish_brake(1.0)
        ros2_interface.publish_accelerator(1.0)
    elif command == "backward":
        ros2_interface.publish_brake(1.0)
        ros2_interface.publish_accelerator(-0.9)
    elif command == "stop":
        ros2_interface.publish_accelerator(0.1)
        ros2_interface.publish_brake(0.0)
    elif command == "left":
        ros2_interface.publish_steering(45.0)
    elif command == "right":
        ros2_interface.publish_steering(135.0)
    elif command == "center":
        ros2_interface.publish_steering(90.0)

    return JsonResponse({"status": "success", "command": command})

# RaceWheel stuff
# New function specifically for controller input
def send_controller_command(request):
    command = request.GET.get('command', '')
    value = float(request.GET.get('value', 1.0))

    if command == "accelerator":
        ros2_interface.publish_accelerator(value)
    elif command == "brake":
        ros2_interface.publish_brake(value)
    elif command == "steering":
        ros2_interface.publish_steering(value)

    return JsonResponse({"status": "success", "command": command, "value": value})

# Video Stuff
import cv2
import time
from django.http import StreamingHttpResponse

# Global camera instances
camera_1 = cv2.VideoCapture(0, cv2.CAP_V4L2)  # /dev/video0
camera_2 = cv2.VideoCapture(2, cv2.CAP_V4L2)  # /dev/video2

def generate_frames(camera, frame_rate=10):
    """Yields frames from the given camera as an MJPEG stream."""
    interval = 1 / frame_rate  # Interval between frames

    while True:
        start_time = time.time()

        success, frame = camera.read()
        if not success:
            break

        # Resize to 480p (smaller size = less data to send)
        frame = cv2.resize(frame, (640, 480))

        # Reduce JPEG quality to 50 (Lower quality = less delay)
        _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 50])

        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
        
        elapsed_time = time.time() - start_time
        if elapsed_time < interval:
            time.sleep(interval - elapsed_time)  # Ensure target FPS

def video_feed_1(request):
    """Video feed for Camera 1 (/dev/video0)."""
    return StreamingHttpResponse(generate_frames(camera_1), content_type='multipart/x-mixed-replace; boundary=frame')

def video_feed_2(request):
    """Video feed for Camera 2 (/dev/video2)."""
    return StreamingHttpResponse(generate_frames(camera_2), content_type='multipart/x-mixed-replace; boundary=frame')

def cleanup():
    """Release camera resources on shutdown."""
    camera_1.release()
    camera_2.release()
