FROM ros:humble-ros-base

SHELL ["/bin/bash", "-c"]

# Install system dependencies including python3-pip
RUN apt-get update && apt-get install -y \
    python3-pip python3-opencv git vim curl python3-colcon-common-extensions python3-rosdep \
    ros-humble-cv-bridge ros-humble-rosbridge-server \
    && rm -rf /var/lib/apt/lists/*

# Upgrade pip and install Python packages
RUN python3 -m pip install --upgrade pip
RUN python3 -m pip install setuptools==65.5.0 packaging==23.0 "numpy<2" wheel torch torchvision torchaudio ultralytics opencv-python \
    python-dotenv websocket-client

# Download MobileCLIP asset for YOLOE promptable models using curl
RUN mkdir -p /root/.cache/torch/hub/checkpoints && \
    curl -L -o /root/.cache/torch/hub/checkpoints/mobileclip_blt.ts \
    https://github.com/ultralytics/assets/releases/download/v0.0.0/mobileclip_blt.ts

# Install additional dependencies for building micro-ROS Agent
RUN apt-get update && apt-get install -y \
    python3-setuptools \
    ros-humble-fastcdr \
    ros-humble-fastrtps \
    ros-humble-fastrtps-cmake-module \
    && rm -rf /var/lib/apt/lists/*

# Clone and build micro-ROS Agent
RUN git clone -b humble https://github.com/micro-ROS/micro-ROS-Agent.git /root/micro-ROS-Agent && \
    git clone -b humble https://github.com/micro-ROS/micro_ros_msgs.git /root/micro-ROS-Agent/src/micro_ros_msgs
WORKDIR /root/micro-ROS-Agent
RUN source /opt/ros/humble/setup.bash && colcon build --symlink-install

# Set up workspace
WORKDIR /root/HandGuestureRobot/ros2_ws

# Source ROS 2 automatically when container starts
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /root/micro-ROS-Agent/install/setup.bash" >> ~/.bashrc

CMD ["/bin/bash"]