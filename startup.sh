#!/bin/bash

# Source ROS2 environment (if needed)
source /opt/ros/jazzy/setup.bash
source /home/pi/Desktop/TeleOpCar/lenv/bin/activate

# Start a new tmux session
tmux new-session -d -s teleop

# Create separate windows for each script
tmux new-window -t teleop:1 -n "Motor" "python3 /home/pi/Desktop/TeleOpCar/src/motors/src/motors_control_node.py"
tmux new-window -t teleop:2 -n "Servo" "python3 /home/pi/Desktop/TeleOpCar/src/motors/src/servo_control_node.py"
tmux new-window -t teleop:3 -n "Brake" "python3 /home/pi/Desktop/TeleOpCar/src/motors/src/brake_manager_node.py"
tmux new-window -t teleop:4 -n "Obstacle" "python3 /home/pi/Desktop/TeleOpCar/src/sensors/launch/obstacle_launch.py"
#tmux new-window -t teleop:5 -n "Server" "python3 /home/pi/Desktop/TeleOpCar/backend/manage.py runserver 0.0.0.0:8000"

# Attach to tmux session (optional, if you want it to open automatically)
tmux attach-session -t teleop