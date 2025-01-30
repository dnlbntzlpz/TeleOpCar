from flask import Flask, render_template, request, jsonify, Response
from motor_control import (
    motor_forward,
    motor_backward,
    motor_stop,
    steer_left,
    steer_right,
    steer_center,
    set_servo_angle
    )
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
    """Handle control commands from the frontend."""
    try:
        data = request.json
        action = data.get("action")
        value = float(data.get("value", 0))  # Use `value` for the servo angle

        # Map commands to functions
        if action == "servo":
            set_servo_angle(value)  # Pass angle to the servo control function
        elif action == "forward":
            motor_forward(value)
        elif action == "backward":
            motor_backward(value)
        elif action == "stop":
            motor_stop()
        elif action == "left":
            steer_left()
        elif action == "right":
            steer_right()
        elif action == "center":
            steer_center()

        return jsonify({"status": "success", "action": action})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 400



@app.route("/video_feed")
def video_feed():
    """Video streaming route."""
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000)
