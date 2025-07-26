# Dockerfile
FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive

# 1. Install necessary dependencies (incl. for ML extensions, e.g., torch; opencv via rosdep)
RUN apt update && apt install -y \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    ros-humble-rosbridge-server \
    ros-humble-cv-bridge \
    ros-humble-foxglove-bridge \
    v4l-utils \
  && pip3 install torch torchvision \
  && rm -rf /var/lib/apt/lists/*

# 2. Initialize and update rosdep for dependencies from package.xml (handle if already init)
RUN rosdep init || true && rosdep update

# 3. Use bash for all following steps
SHELL ["/bin/bash", "-lc"]

# 4. Copy code, install dependencies with rosdep (skip ament_python since no rosdep key), then build with colcon
WORKDIR /root/ros2_ws
COPY src ./src
RUN rosdep install -i --from-path src --rosdistro humble -y --skip-keys ament_python
RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install

# 5. Copy entrypoint and make it executable
COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

# 6. Automatic start: Run foxglove_bridge on 8765 AND the stream node
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash", "-c", "ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765 & ros2 run stream2ros stream_node"]