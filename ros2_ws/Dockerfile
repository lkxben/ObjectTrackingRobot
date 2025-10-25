# Use ROS 2 Humble base image
FROM ros:humble-ros-base

# Install system dependencies
RUN apt-get update && apt-get install -y \
    python3-pip python3-opencv git vim curl \
    python3-colcon-common-extensions python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# Upgrade pip
RUN pip3 install --upgrade pip setuptools wheel

# Install ML / Python dependencies
RUN pip3 install torch torchvision torchaudio ultralytics opencv-python numpy

# Set up workspace
WORKDIR /root/ros2_ws

# Source ROS 2 automatically when container starts
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

WORKDIR /root/ros2_ws
CMD ["/bin/bash"]