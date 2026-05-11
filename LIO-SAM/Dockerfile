FROM osrf/ros:humble-desktop-full-jammy

RUN apt-get update \
    && apt-get install -y \
        python3-colcon-common-extensions \
        software-properties-common \
        ros-humble-navigation2 \
        ros-humble-robot-localization \
        ros-humble-robot-state-publisher \
        ros-humble-perception-pcl \
        ros-humble-pcl-msgs \
        ros-humble-vision-opencv \
        ros-humble-xacro \
        ros-humble-rmw-cyclonedds-cpp \
    && add-apt-repository -y ppa:borglab/gtsam-release-4.1 \
    && apt-get update \
    && apt-get install -y \
        libgtsam-dev \
        libgtsam-unstable-dev \
    && rm -rf /var/lib/apt/lists/*

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

SHELL ["/bin/bash", "-c"]

RUN mkdir -p /root/ros2_ws/src/LIO-SAM
COPY . /root/ros2_ws/src/LIO-SAM

RUN cd /root/ros2_ws \
    && source /opt/ros/humble/setup.bash \
    && colcon build

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc \
    && echo "source /root/ros2_ws/install/setup.bash" >> /root/.bashrc \
    && echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /root/.bashrc

WORKDIR /root/ros2_ws