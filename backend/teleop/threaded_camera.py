import cv2
import threading
import time
from turbojpeg import TurboJPEG

jpeg = TurboJPEG()

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
        target_fps = 15  # Set your desired capture frame rate
        frame_time = 1.0 / target_fps
        
        while self.running:
            loop_start = time.time()
            
            if self.camera.isOpened():
                self.camera.grab()  # Discard stale frame
                success, frame = self.camera.read()
                if success:
                    with self.lock:
                        self.latest_frame = frame
            
            # Calculate time spent in this iteration
            processing_time = time.time() - loop_start
            
            # Sleep to maintain desired frame rate
            sleep_time = max(0, frame_time - processing_time)
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                # If we're running behind, give the system a tiny breather
                time.sleep(0.001)

    def get_frame(self):
        with self.lock:
            if self.latest_frame is not None:
                return jpeg.encode(self.latest_frame, quality=30)
            return None

    def stop(self):
        self.running = False
        if self.thread.is_alive():
            self.thread.join(timeout=1.0)
        self.camera.release()
