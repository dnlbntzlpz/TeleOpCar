# Teledriving Car Project

This project enables real-time remote control of a toy car using a **Logitech G923 racing wheel**, a **Raspberry Pi 4B**, and **ROS2**. The car features a dual-motor drive controlled via **IBT-4 H-bridges**, front-wheel steering with a servo motor, and live video streaming with ultra-low latency. A web interface built with **Django** allows for teleoperation, command history, and future dashboard telemetry.

---

## 🚦 Features

- 🔧 Hardware Control (motors + servo) via Raspberry Pi 4B
- 🧠 ROS2 Jazzy for real-time control & sensor integration
- 🎮 Logitech G923 Racing Wheel Support
- 🌐 Low-latency Web UI using Django & WebSockets
- 📷 Dual camera streaming with threaded OpenCV

---

## 🧰 Hardware Used

- Raspberry Pi 4B (Ubuntu 24.04)
- 2x IBT-4 H-bridge motor drivers
- Steering Servo (e.g., RDS5160)
- Logitech G923 Racing Wheel
- Dual USB Cameras
- 5V/12V variable power supply
- Optional: Ultrasonic Sensors

---

## 🛠️ Software Stack

- Python 3.10+
- ROS2 Jazzy
- Django 5.1.5
- OpenCV (for camera streaming)
- gpiozero (for GPIO handling)

---

## 🚀 Setup Guide

### 1. Flash Ubuntu 24.04 on the Raspberry Pi 5

- [Download Ubuntu Server 24.04](https://ubuntu.com/download/raspberry-pi)
- Flash using [Raspberry Pi Imager](https://www.raspberrypi.com/software/)

### 2. Clone the Repository

On the Raspberry Pi:

```bash
git clone https://github.com/dnlbntzlpz/TeleOpCar.git
cd teledriving-car
```

### 3. Run the Startup Script

This script launches all required ROS2 nodes, the Django server, and the Cloudflare tunnel (if configured):

```
chmod +x startup.sh
./startup.sh
```

### 4. Connect to the Web Interface

Once the script is running, open your browser on the Raspberry Pi and navigate to:

```
http://localhost:8071
```

You should now see the live control dashboard where you can drive the car using:
```
    W, A, S, D to move

    X to stop

    C to center the steering servo

    Or use the Logitech G923 racing wheel (shifters and pedals)
```