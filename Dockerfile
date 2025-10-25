FROM ros:jazzy

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=jazzy \
    LANG=C.UTF-8

RUN apt-get update && apt-get install -y \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-slam-toolbox \
    ros-jazzy-tf-transformations \
    python3-pip \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y python3-venv
    
RUN python3 -m venv /opt/venv

ENV PATH="/opt/venv/bin:$PATH"

RUN pip install pyserial numpy

RUN rosdep update

WORKDIR /ros2_ws

RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]