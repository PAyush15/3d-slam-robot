# 3d-slam-robot

# Refresh expired ROS GPG key (non-interactive & modern way)
RUN apt-get update && apt-get install -y curl gnupg2 ca-certificates \
 && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg \
 && echo "deb [arch=arm64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" \
    > /etc/apt/sources.list.d/ros2.list \
 && apt-get update

