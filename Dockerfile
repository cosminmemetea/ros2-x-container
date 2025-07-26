# Dockerfile
FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive

# 1. Instalează dependențele necesare (incl. pentru ML extensibil, e.g., torch; opencv via rosdep)
RUN apt update && apt install -y \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    ros-humble-rosbridge-server \
    ros-humble-cv-bridge \
    v4l-utils \
  && pip3 install torch torchvision \
  && rm -rf /var/lib/apt/lists/*

# 2. Inițializează și actualizează rosdep pentru dependențe din package.xml (handle if already init)
RUN rosdep init || true && rosdep update

# 3. Lucrează cu bash pentru toți pașii următori
SHELL ["/bin/bash", "-lc"]

# 4. Copiază codul, instalează dependențe cu rosdep (skip ament_python since no rosdep key), apoi build cu colcon
WORKDIR /root/ros2_ws
COPY src ./src
RUN rosdep install -i --from-path src --rosdistro humble -y --skip-keys ament_python
RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install

# 5. Copiază entrypoint-ul și fă-l executabil
COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

# 6. Pornire automată: Rulează rosbridge pe 8765 ȘI nodul stream
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash", "-c", "ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=8765 & ros2 run stream2ros stream_node"]