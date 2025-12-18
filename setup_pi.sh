#!/bin/bash
set -e

echo "=== Updating OS ==="
sudo apt update
sudo apt upgrade -y

echo "=== Installing system dependencies ==="
sudo apt install -y \
    python3-pip python3-opencv git vim curl python3-colcon-common-extensions python3-rosdep \
    ros-humble-cv-bridge ros-humble-rosbridge-server \
    python3-lgpio python3-pigpio python3-rpi.gpio \
    libraspberrypi0 libraspberrypi-dev \
    libatlas-base-dev build-essential

echo "=== Upgrading pip and installing Python packages ==="
python3 -m pip install --upgrade pip
python3 -m pip install setuptools==65.5.0 packaging==23.0 "numpy<1.24" wheel torch torchvision torchaudio \
    ultralytics opencv-python python-dotenv websocket-client
python3 -m pip install --user gpiozero

echo "=== Enabling pigpio daemon ==="
sudo systemctl enable pigpiod
sudo systemctl start pigpiod

echo "=== Creating ROS 2 workspace ==="
mkdir -p ~/ObjectTrackingRobot/ros2_ws/src
cd ~/ObjectTrackingRobot/ros2_ws

echo "=== Cloning ByteTrack ==="
git clone https://github.com/ifzhang/ByteTrack.git ~/ByteTrack
cd ~/ByteTrack
python3 -m pip install onnxruntime==1.22.0
python3 -m pip install loguru scikit-image tqdm torchvision Pillow thop ninja tabulate tensorboard lap motmetrics filterpy h5py cython_bbox cython "numpy<1.24"
python3 setup.py develop

echo "=== Cloning DaSiamRPN ==="
cd ~
git clone https://github.com/foolwood/DaSiamRPN.git ~/DaSiamRPN
cd ~/DaSiamRPN
python3 -m pip install gdown
gdown --id 1-vNVZxfbIplXHrqMHiJJYWXYWsOIvGsf -O ~/DaSiamRPN/code/SiamRPNBIG.model

echo "=== Download MobileCLIP asset ==="
mkdir -p ~/.cache/ultralytics/assets
curl -L -o ~/.cache/ultralytics/assets/mobileclip_blt.ts \
    https://github.com/ultralytics/assets/releases/download/v0.0.0/mobileclip_blt.ts

echo "=== Building ROS 2 workspace ==="
cd ~/ObjectTrackingRobot/ros2_ws
source /opt/ros/humble/setup.bash
colcon build

echo "=== Setting up automatic ROS sourcing ==="
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/ObjectTrackingRobot/ros2_ws/install/setup.bash" >> ~/.bashrc

echo "=== Setup complete! ==="
echo "Reboot or run 'source ~/.bashrc' to start using ROS and Python environment."