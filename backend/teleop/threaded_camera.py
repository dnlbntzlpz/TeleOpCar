import cv2
import threading
import time

class ThreadedCamera:
    def __init__(self, camera_index):
        self.camera = cv2.VideoCapture(camera_index, cv2.CAP_V4L2)
        self.camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        
        # Lower resolution for better performance
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        
        self.latest_frame = None
        self.lock = threading.Lock()
        self.running = True

        # Start the frame capture thread
        self.thread = threading.Thread(target=self.update_frame, daemon=True)
        self.thread.start()

    def update_frame(self):
        while self.running:
            if not self.camera.isOpened():
                print(f"Error: Camera not opened")
                time.sleep(0.1)
                continue
            
            success, frame = self.camera.read()
            if success:
                # Directly encode the frame without resizing
                _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 30])
                
                with self.lock:
                    self.latest_frame = buffer.tobytes()
            
            # Small sleep to prevent CPU overload
            time.sleep(0.01)

    def get_frame(self):
        with self.lock:
            return self.latest_frame

    def stop(self):
        self.running = False
        if self.thread.is_alive():
            self.thread.join(timeout=1.0)
        if self.camera.isOpened():
            self.camera.release()
