#!/bin/bash

# Source ROS2 environment (if needed)
source /opt/ros/jazzy/setup.bash
source /home/pi/Desktop/TeleOpCar/lenv/bin/activate

# Start a new tmux session
tmux new-session -d -s teleop

# Create separate windows for each script
#Motor
tmux new-window -t teleop:1 -n "Motor" "python3 /home/pi/Desktop/TeleOpCar/src/motors/src/motors_control_node.py"
sleep 2  # 2-second delay

#Servo
tmux new-window -t teleop:2 -n "Servo" "python3 /home/pi/Desktop/TeleOpCar/src/motors/src/servo_control_node.py"
sleep 2  # 2-second delay

#Ultrasonics
#tmux new-window -t teleop:3 -n "Obstacle" "python3 /home/pi/Desktop/TeleOpCar/src/sensors/src/obstacleDetection_node.py"
sleep 2  # 2-second delay

#Brake node
tmux new-window -t teleop:4 -n "Brake" "python3 /home/pi/Desktop/TeleOpCar/src/motors/src/brake_manager_node.py"
sleep 2  # 2-second delay

#Server
tmux new-window -t teleop:5 -n "Server" "python3 /home/pi/Desktop/TeleOpCar/backend/manage.py runserver 192.168.10.128:8071"
sleep 5  # 5-second delay

#Ngrok tunnel
#tmux new-window -t teleop:6 -n "Ngrok" "ngrok http http://192.168.10.128:8071"
#sleep 2  # 2-second delay

# Attach to tmux session (optional, if you want it to open automatically)
tmux attach-session -t teleop
