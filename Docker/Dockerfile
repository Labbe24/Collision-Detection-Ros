# Structure copied from https://hub.docker.com/_/ros
ARG FROM_IMAGE=ros:galactic
ARG OVERLAY_WS=/opt/ros/overlay_ws

# multi-stage for caching
FROM $FROM_IMAGE AS cacher

# clone overlay source
ARG OVERLAY_WS
WORKDIR $OVERLAY_WS/src

RUN echo "\
repositories: \n\
  collision_detection: \n\
    type: git \n\
    url: https://github.com/Labbe24/Collision-Detection-Ros.git \n\
  Universal_Robots_ROS2_Description: \n\
    type: git \n\
    url: https://github.com/UniversalRobots/Universal_Robots_ROS2_Description.git \n\
    version: ros2 \n\
  fri: \n\
    type: git \n\
    url: https://github.com/KCL-BMEIS/fri \n\
    version: ros2_hwi-dev \n\
  lbr_fri_ros2_stack: \n\
    type: git \n\
    url: https://github.com/KCL-BMEIS/lbr_fri_ros2_stack \n\
    version: galactic \n\
  tcp_endpoint: \n\
    type: git \n\
    url: https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git \n\
    version: main-ros2 \n\
" > ../overlay.repos
RUN vcs import ./ < ../overlay.repos

# copy manifests for caching
WORKDIR /opt
RUN mkdir -p /tmp/opt && \
    find ./ -name "package.xml" | \
      xargs cp --parents -t /tmp/opt && \
    find ./ -name "COLCON_IGNORE" | \
      xargs cp --parents -t /tmp/opt || true

# multi-stage for building
FROM $FROM_IMAGE AS builder

# install overlay dependencies
ARG OVERLAY_WS
WORKDIR $OVERLAY_WS
COPY --from=cacher /tmp/$OVERLAY_WS/src ./src
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    apt-get update && \
    rosdep install --ignore-src \
    --from-paths src/collision_detection \
    src/Universal_Robots_ROS2_Description \
    src/lbr_fri_ros2_stack/lbr_description \
    src/tcp_endpoint -y -r \
    && rm -rf /var/lib/apt/lists/*

# build overlay source
COPY --from=cacher $OVERLAY_WS/src ./src
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build --packages-select ros_tcp_endpoint collision_detection_msgs ur_description lbr_description collision_detection_launch\
    --cmake-args -DCMAKE_BUILD_TYPE=Release
# source entrypoint setup
ENV OVERLAY_WS $OVERLAY_WS
RUN sed --in-place --expression \
      '$isource "$OVERLAY_WS/install/setup.bash"' \
      /ros_entrypoint.sh