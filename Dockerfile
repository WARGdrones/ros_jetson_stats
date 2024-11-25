FROM ros:noetic-ros-core-focal

# install pip3
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3 \
    python3-pip \
    ros-noetic-diagnostic-aggregator \
    build-essential \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

#upgrade pip
RUN python3 -m pip install --upgrade pip
#install jetson-stats
RUN python3 -m pip install -U jetson-stats

#source ros env
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc