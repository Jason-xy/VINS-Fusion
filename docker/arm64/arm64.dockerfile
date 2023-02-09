FROM jasonxxxyyy/sky-explorer:runtime-cuda11.4-ros2-arm64

RUN rm /bin/sh && \
    ln -s /bin/bash /bin/sh

ENV ROS2_WS=/root/ros2_ws
COPY ./ $ROS2_WS/src/
RUN cd $ROS2_WS && \
    source /opt/ros/foxy/setup.bash && \
    colcon build --symlink-install --allow-overriding cv_bridge --cmake-args -DCMAKE_BUILD_TYPE=Release

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    python3-pip && \
    python3 -m pip install --upgrade pip && \
    python3 -m pip install rosbags

