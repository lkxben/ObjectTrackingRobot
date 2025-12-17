FROM ros:humble-ros-base

SHELL ["/bin/bash", "-c"]

# Install system dependencies including python3-pip
RUN apt-get update && apt-get install -y \
python3-pip python3-opencv git vim curl python3-colcon-common-extensions python3-rosdep \
ros-humble-cv-bridge ros-humble-rosbridge-server \
&& rm -rf /var/lib/apt/lists/*

# Upgrade pip and install Python packages
RUN python3 -m pip install --upgrade pip
RUN python3 -m pip install setuptools==65.5.0 packaging==23.0 "numpy<1.24" wheel torch torchvision torchaudio ultralytics opencv-python \
python-dotenv websocket-client

# Download MobileCLIP asset for YOLOE promptable models using curl
RUN mkdir -p /root/.cache/ultralytics/assets && \
curl -L -o /root/.cache/ultralytics/assets/mobileclip_blt.ts \
https://github.com/ultralytics/assets/releases/download/v0.0.0/mobileclip_blt.ts

# Set up workspace
WORKDIR /root/ObjectTrackingRobot/ros2_ws

RUN chmod +x setup_ros.sh

# Install ByteTrack
RUN git clone https://github.com/ifzhang/ByteTrack.git /root/ByteTrack
WORKDIR /root/ByteTrack
RUN python3 -m pip install onnxruntime==1.22.0
RUN python3 -m pip install loguru scikit-image tqdm torchvision Pillow thop ninja tabulate tensorboard lap motmetrics filterpy h5py cython_bbox lap cython "numpy<1.24"
RUN python3 setup.py develop

# # Install LightTrack
# WORKDIR /root
# RUN git clone https://github.com/researchmm/LightTrack.git /root/LightTrack
# WORKDIR /root/LightTrack
# RUN python3 -m pip install --no-deps torch torchvision opencv-python yacs scipy easydict "numpy<1.24"

# # Install NanoTrack
# WORKDIR /root
# RUN python3 -m pip install --no-deps nanotrack

# Install DaSiamRPN
WORKDIR /root
RUN git clone https://github.com/foolwood/DaSiamRPN.git /root/DaSiamRPN
WORKDIR /root/DaSiamRPN
RUN pip install gdown
RUN gdown --id 1-vNVZxfbIplXHrqMHiJJYWXYWsOIvGsf -O /root/DaSiamRPN/code/SiamRPNBIG.model

# # Source ROS 2 automatically when container starts
# RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

CMD ["/bin/bash", "-c", "bash /ros2_ws/setup_ros.sh && bash"]