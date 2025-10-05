# 3d-slam-robot

# ---- Fix for expired ROS 2 GPG key (2024–2025) ----
RUN apt-get update && apt-get install -y curl gnupg2 ca-certificates && \
    rm -f /etc/apt/sources.list.d/ros2-latest.list || true && \
    echo "deb [arch=arm64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" \
      | tee /etc/apt/sources.list.d/ros2-latest.list && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    apt-get update
