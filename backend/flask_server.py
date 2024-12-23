from flask import Flask, render_template, request, jsonify
from motor_control import motor_forward, motor_backward, motor_stop
import os

app = Flask(
    __name__,
    template_folder="../frontend/templates",
    static_folder="../frontend/static")

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

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000)
