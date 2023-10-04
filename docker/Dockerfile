#https://github.com/Maik13579/tiago-sim
FROM tiago-sim:noetic as base

RUN apt update && apt install -y\
  python3-pip \
  ffmpeg libsm6 libxext6 \
  ros-$ROS_DISTRO-tf \
  ros-$ROS_DISTRO-tf2-sensor-msgs \
  ros-$ROS_DISTRO-tf2-geometry-msgs \
&& pip3 install --upgrade open3d numpy compas scipy python-dateutil\
&& rm -rf /var/lib/apt/lists/*
RUN pip3 install opencv-python

# Clone FlexBE
WORKDIR /root/catkin_ws/src
RUN git clone -b upstream/1.3.1 https://github.com/FlexBE/flexbe_behavior_engine-release.git
RUN git clone -b noetic https://github.com/FlexBE/flexbe_app.git

# Install deps
ARG ROSDEP_IGNORE="urdf_test omni_drive_controller orocos_kdl pal_filters libgazebo9-dev pal_usb_utils speed_limit_node camera_calibration_files pal_moveit_plugins pal_startup_msgs pal_local_joint_control pal_pcl_points_throttle_and_filter current_limit_controller hokuyo_node dynamixel_cpp pal_moveit_capabilities pal_pcl dynamic_footprint gravity_compensation_controller pal-orbbec-openni2 pal_loc_measure pal_map_manager ydlidar_ros_driver"
WORKDIR /root/catkin_ws
RUN apt update \
 && rosdep install --from-paths src --ignore-src -r -y --rosdistro $ROS_DISTRO --skip-keys="${ROSDEP_IGNORE}" \
 && rm -rf /var/lib/apt/lists/*

# Get source code
COPY flexbe /root/catkin_ws/src/flexbe
COPY ros /root/catkin_ws/src/ros

FROM base as build
WORKDIR /root/catkin_ws
RUN source /opt/ros/$ROS_DISTRO/setup.bash \
 && catkin_make\
 && echo 'source /root/catkin_ws/devel/setup.bash' >> /root/.bashrc