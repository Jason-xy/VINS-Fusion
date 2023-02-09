FROM jasonxxxyyy/sky-explorer:runtime-cuda11.4-ros2-amd64

ENV ROS2_WS=/root/ros2_ws
COPY ./ $ROS2_WS/src/
RUN cd $ROS2_WS && \
    source /opt/ros/foxy/setup.bash && \
    colcon build --symlink-install --allow-overriding cv_bridge --cmake-args -DCMAKE_BUILD_TYPE=Release

