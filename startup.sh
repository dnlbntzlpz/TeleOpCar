#!/bin/bash

# Source ROS2 and virtual environment
source /opt/ros/jazzy/setup.bash
source /home/pi/Desktop/TeleOpCar/lenv/bin/activate

# Ensure log directory exists
mkdir -p /home/pi/Desktop/TeleOpCar/logs

# Get IP address (for reference)
IP_ADDRESS=$(hostname -I | awk '{print $1}')
echo "Starting services on IP: $IP_ADDRESS"

# Start Motor Control Node
echo "Starting motor control..."
python3 /home/pi/Desktop/TeleOpCar/src/motors/src/motors_control_node.py > logs/motor.log 2>&1 &

# Start Servo Control Node
echo "Starting servo control..."
python3 /home/pi/Desktop/TeleOpCar/src/motors/src/servo_control_node.py > logs/servo.log 2>&1 &

# Start Brake Manager Node
echo "Starting brake manager..."
python3 /home/pi/Desktop/TeleOpCar/src/motors/src/brake_manager_node.py > logs/brake.log 2>&1 &

# Optional: Start Ultrasonic Obstacle Detection Node
# echo "Starting ultrasonic sensor node..."
# python3 /home/pi/Desktop/TeleOpCar/src/sensors/src/obstacleDetection_node.py > logs/ultrasonic.log 2>&1 &

# Start Django server
echo "Starting Django server..."
python3 /home/pi/Desktop/TeleOpCar/backend/manage.py runserver 0.0.0.0:8071 > logs/server.log 2>&1 &

# Start Cloudflare Tunnel
echo "Starting Cloudflare tunnel..."
cloudflared tunnel --url http://127.0.0.1:8071 > logs/cloudflared.log 2>&1 &

echo "✅ All services started. You can now access the web interface at http://localhost:8071"
