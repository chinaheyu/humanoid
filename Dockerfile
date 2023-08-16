FROM ros:foxy

ENV OVERLAY_WS=/opt/ros/overlay_ws

COPY . $OVERLAY_WS/src

WORKDIR $OVERLAY_WS
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    apt-get update && \
    apt-get install -y python3-pip && \
    ./src/install_dependency.sh && \
    rm -rf /var/lib/apt/lists/*

RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build --mixin "release"

ENV OVERLAY_WS $OVERLAY_WS
RUN sed --in-place --expression \
      '$isource "$OVERLAY_WS/install/setup.bash"' \
      /ros_entrypoint.sh

CMD ["ros2", "launch", "humanoid_base", "humanoid_base.py"]
