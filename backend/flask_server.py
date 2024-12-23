from flask import Flask, render_template, request, jsonify, Response
from motor_control import motor_forward, motor_backward, motor_stop
import cv2
import os

app = Flask(
    __name__,
    template_folder="../frontend/templates",
    static_folder="../frontend/static")

# Initialize the camera
camera = cv2.VideoCapture(0)

def generate_frames():
    """Generate frames for the camera feed."""
    while True:
        success, frame = camera.read()
        if not success:
            break
        else:
            # Encode the frame as JPEG
            ret, buffer = cv2.imencode('.jpg', frame)
            frame = buffer.tobytes()

            # Yield the frame as part of a multipart HTTP response
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')


@app.route("/")
def index():
    """Serve the dashboard HTML."""
    return render_template("index.html")

@app.route("/control", methods=["POST"])
def control():
    """Handle control commands sent from the frontend."""
    data = request.json
    action = data.get("action")
    speed = data.get("speed", 0.9)  # Default speed

    if action == "forward":
        motor_forward(speed)
    elif action == "backward":
        motor_backward(speed)
    elif action == "stop":
        motor_stop()

    return jsonify({"status": "success", "action": action})

@app.route("/video_feed")
def video_feed():
    """Video streaming route."""
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000)
