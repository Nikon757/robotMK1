# RobotMK1 RTAB-Map stack: ROS 2 Jazzy + rtabmap_ros + depthai_ros (OAK-D) + LDROBOT driver (STL-27L)
#
#   docker build -t robotmk1/rtabmap:jazzy .          (~20-40 min on a Pi 5, ~3 GB image)
#
# The launch file, parameters and robot.yaml are bind-mounted at run time (see run_rtabmap.py),
# so editing them does NOT require a rebuild.
FROM ros:jazzy-ros-base

ARG DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8 \
    ROS_DISTRO=jazzy \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-rtabmap-ros \
        ros-jazzy-depthai-ros \
        ros-jazzy-imu-filter-madgwick \
        ros-jazzy-nav2-map-server \
        ros-jazzy-rmw-cyclonedds-cpp \
        ros-jazzy-tf2-ros ros-jazzy-tf2-tools \
        ros-jazzy-rosbag2 ros-jazzy-rosbag2-storage-mcap \
        python3-colcon-common-extensions python3-yaml python3-numpy \
        git build-essential cmake usbutils udev nano \
    && rm -rf /var/lib/apt/lists/*

# LDROBOT LD06 / LD19 / STL-27L driver (not in the ROS apt repo) - built from source
WORKDIR /ws
RUN mkdir -p src \
    && git clone --depth 1 https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2.git src/ldlidar_stl_ros2 \
    && . /opt/ros/jazzy/setup.sh \
    && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && rm -rf build log

# OAK udev rule inside the container is harmless; the host rule is what matters.
RUN echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' > /etc/udev/rules.d/80-movidius.rules

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
WORKDIR /robot
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
