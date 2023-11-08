FROM ros:foxy

ENV OVERLAY_WS=/opt/ros/overlay_ws

COPY . $OVERLAY_WS/src

WORKDIR $OVERLAY_WS
RUN ./src/install_dependency.sh && \
    rm -rf /var/lib/apt/lists/*

RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build --mixin "release"

ENV OVERLAY_WS $OVERLAY_WS
RUN sed --in-place --expression \
      '$isource "$OVERLAY_WS/install/setup.bash"' \
      /ros_entrypoint.sh

ENV ROS_DOMAIN_ID=1
CMD ["ros2", "launch", "humanoid_bringup", "bringup.py"]
