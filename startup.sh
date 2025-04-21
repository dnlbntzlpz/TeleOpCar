#!/bin/bash

# Source environments
source /opt/ros/jazzy/setup.bash
source /home/pi/Desktop/TeleOpCar/lenv/bin/activate

# Wait for Wi-Fi or network (checks until internet is available)
echo "🔍 Waiting for internet connection..."
while ! ping -c 1 -W 1 8.8.8.8; do
  sleep 1
done
echo "✅ Internet connection detected."

# Wait for cameras or USB devices (adjust timing if needed)
echo "⏳ Waiting for USB devices to initialize..."
sleep 10

# Ensure log directory exists
mkdir -p /home/pi/Desktop/TeleOpCar/logs

# Track background PIDs
PIDS=()

# Cleanup function to run on exit
cleanup() {
  echo "⛔ Shutting down all processes..."
  for pid in "${PIDS[@]}"; do
    kill "$pid" 2>/dev/null
  done
  echo "✅ All background processes terminated."
  exit 0
}
trap cleanup EXIT INT

# Start Motor Control Node
python3 /home/pi/Desktop/TeleOpCar/src/motors/src/motors_control_node.py > logs/motor.log 2>&1 &
PIDS+=($!)

# Start Servo Control Node
python3 /home/pi/Desktop/TeleOpCar/src/motors/src/servo_control_node.py > logs/servo.log 2>&1 &
PIDS+=($!)

# Start Brake Manager Node
python3 /home/pi/Desktop/TeleOpCar/src/motors/src/brake_manager_node.py > logs/brake.log 2>&1 &
PIDS+=($!)

# Optional: Ultrasonic Sensor
# python3 /home/pi/Desktop/TeleOpCar/src/sensors/src/obstacleDetection_node.py > logs/ultrasonic.log 2>&1 &
# PIDS+=($!)

# Start Django Server
python3 /home/pi/Desktop/TeleOpCar/backend/manage.py runserver 0.0.0.0:8071 > logs/server.log 2>&1 &
PIDS+=($!)

# Start Cloudflare Tunnel
#cloudflared tunnel --url http://127.0.0.1:8071 > logs/cloudflared.log 2>&1 &
#PIDS+=($!)

echo "🚀 All services launched at http://localhost:8071"
echo "🔁 Press Ctrl+C to shut everything down."

# Wait for Ctrl+C
while true; do
  sleep 1
done
