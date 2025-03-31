from django.shortcuts import render
from django.http import JsonResponse
from .ros_interface import ROS2Interface
import rclpy
from django.http import StreamingHttpResponse
from .threaded_camera import ThreadedCamera
import time

# Ensure rclpy is initialized only once
if not rclpy.ok():
    rclpy.init()

ros2_interface = ROS2Interface()

# Initialize cameras
camera_1 = ThreadedCamera(camera_index=0, fps=15)
camera_2 = ThreadedCamera(camera_index=2, fps=15)

def index(request):
    return render(request, 'teleop/index.html')

def send_command(request):
    command = request.GET.get('command', '')
    value = float(request.GET.get('value', '1.0'))  # Default to 1.0 if no value provided

    if command == "forward":
        ros2_interface.publish_brake(0.0)
        ros2_interface.publish_accelerator(value)
    elif command == "backward":
        ros2_interface.publish_brake(0.0)
        ros2_interface.publish_accelerator(-value + 0.1)
    elif command == "stop":
        ros2_interface.publish_accelerator(0.1)
        ros2_interface.publish_brake(1.0)
    elif command == "left":
        # Use the value parameter as the steering angle
        # Calculate the angle: 90 - value (to get angle to the left of center)
        ros2_interface.publish_steering(90 - value)
    elif command == "right":
        # Use the value parameter as the steering angle
        # Calculate the angle: 90 + value (to get angle to the right of center)
        ros2_interface.publish_steering(90 + value)
    elif command == "center":
        ros2_interface.publish_steering(90.0)

    return JsonResponse({"status": "success", "command": command})

# RaceWheel stuff
# New function specifically for controller input
def send_controller_command(request):
    command = request.GET.get('command', '')
    value = float(request.GET.get('value', ''))

    if command == "accelerator":
        if value >= 0 and (abs(value) <= (.3)):
            ros2_interface.publish_accelerator(0.1)
            ros2_interface.publish_brake(1.0)
        else:
            ros2_interface.publish_brake(0.0)
            ros2_interface.publish_accelerator(value)
    elif command == "brake":
        ros2_interface.publish_brake(value)
    elif command == "steering":
        ros2_interface.publish_steering(value)

    return JsonResponse({"status": "success", "command": command, "value": value})

# Video Stuff
import cv2
import time

def video_feed_1(request):
    def generate():
        frame_count = 0
        start_time = time.time()
        target_fps = 15  # Set your desired frame rate here
        frame_time = 1.0 / target_fps
        
        while True:
            try:
                loop_start = time.time()
                
                frame = camera_1.get_frame()
                if frame:
                    frame_count += 1
                    # Print diagnostics every 50 frames
                    if frame_count % 50 == 0:
                        elapsed = time.time() - start_time
                        #print(f"Camera 1: {frame_count/elapsed:.2f} FPS, frame size: {len(frame)} bytes")
                    
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
                else:
                    print("Camera 1: No frame received")
                    time.sleep(0.01)  # Short sleep if no frame
                    continue  # Skip the rate limiting for this iteration
                
                # Calculate time spent in this iteration
                processing_time = time.time() - loop_start
                
                # Sleep to maintain desired frame rate
                sleep_time = max(0, frame_time - processing_time)
                if sleep_time > 0:
                    time.sleep(sleep_time)
                    
            except Exception as e:
                print(f"Camera 1 error: {str(e)}")
                time.sleep(0.1)  # Longer sleep on error

    response = StreamingHttpResponse(generate(), content_type='multipart/x-mixed-replace; boundary=frame')
    response['Cache-Control'] = 'no-cache, no-store, must-revalidate'
    response['Pragma'] = 'no-cache'
    response['Expires'] = '0'
    return response

def video_feed_2(request):
    def generate():
        frame_count = 0
        start_time = time.time()
        target_fps = 15  # Set your desired frame rate here
        frame_time = 1.0 / target_fps
        
        while True:
            try:
                loop_start = time.time()
                
                frame = camera_2.get_frame()
                if frame:
                    frame_count += 1
                    # Print diagnostics every 50 frames
                    if frame_count % 50 == 0:
                        elapsed = time.time() - start_time
                        #print(f"Camera 2: {frame_count/elapsed:.2f} FPS, frame size: {len(frame)} bytes")
                    
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
                else:
                    print("Camera 2: No frame received")
                    time.sleep(0.01)  # Short sleep if no frame
                    continue  # Skip the rate limiting for this iteration
                
                # Calculate time spent in this iteration
                processing_time = time.time() - loop_start
                
                # Sleep to maintain desired frame rate
                sleep_time = max(0, frame_time - processing_time)
                if sleep_time > 0:
                    time.sleep(sleep_time)
                    
            except Exception as e:
                print(f"Camera 2 error: {str(e)}")
                time.sleep(0.1)  # Longer sleep on error

    response = StreamingHttpResponse(generate(), content_type='multipart/x-mixed-replace; boundary=frame')
    response['Cache-Control'] = 'no-cache, no-store, must-revalidate'
    response['Pragma'] = 'no-cache'
    response['Expires'] = '0'
    return response

def cleanup():
    """Release camera resources on shutdown."""
    camera_1.stop()
    camera_2.stop()

# Telemetry Stuff
import random
from django.http import JsonResponse

def get_mock_telemetry(request):
    """Returns random but reasonable telemetry values for testing."""
    telemetry_data = {
        "temperature": round(random.uniform(30, 80), 1),  # 30-80°C
        "battery": random.randint(50, 100),  # 50-100%
        "latency": random.randint(50, 200),  # 50-200ms
        "fps": random.randint(5, 30),  # 5-30 FPS
        "connection": "Connected",
        "signal": random.randint(50, 100),  # 50-100%
        "gamepad": "Connected" if random.random() > 0.3 else "No Gamepad Detected",  # 70% chance of being connected
        "cpu_load": round(random.uniform(10, 80), 1),  # 10-80% CPU load
    }
    return JsonResponse(telemetry_data)
