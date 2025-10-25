FROM ros:humble-ros-base

# Install system dependencies including python3-pip
RUN apt-get update && apt-get install -y \
    python3-pip python3-opencv git vim curl python3-colcon-common-extensions python3-rosdep \
    ros-humble-cv-bridge \
    && rm -rf /var/lib/apt/lists/*

# Upgrade pip and install Python packages
RUN python3 -m pip install --upgrade pip
RUN python3 -m pip install setuptools==65.5.0 packaging==23.0 "numpy<2" wheel torch torchvision torchaudio ultralytics opencv-python

# Set up workspace
WORKDIR /root/HandGuestureRobot/ros2_ws

# Source ROS 2 automatically when container starts
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

CMD ["/bin/bash"]