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
        ros2_interface.publish_brake(0.0)
    elif command == "left":
        ros2_interface.publish_steering(45.0)
    elif command == "right":
        ros2_interface.publish_steering(135.0)
    elif command == "center":
        ros2_interface.publish_steering(90.0)

    return JsonResponse({"status": "success", "command": command})

# Video Streaming
import cv2
from django.http import StreamingHttpResponse

def generate_frames(camera_index):
    """Captures frames from a specified camera index and serves as MJPEG."""
    camera = cv2.VideoCapture(camera_index)
    while True:
        success, frame = camera.read()
        if not success:
            break
        else:
            _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
            frame = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
    camera.release()

def video_feed_1(request):
    """Handles the first camera video feed."""
    return StreamingHttpResponse(generate_frames(0),
                                 content_type='multipart/x-mixed-replace; boundary=frame')

def video_feed_2(request):
    """Handles the second camera video feed."""
    return StreamingHttpResponse(generate_frames(2),
                                 content_type='multipart/x-mixed-replace; boundary=frame')
