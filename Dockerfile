FROM ros:jazzy-ros-base-noble

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-jazzy-rosbag2 \
    ros-jazzy-tf2-tools \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
CMD ["bash"]

\n
