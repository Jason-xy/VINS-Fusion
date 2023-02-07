#!/bin/bash
sudo docker run --rm \
    -v `pwd`:/root/ros2_ws/src \
    jasonxxxyyy/sky-explorer:runtime-cuda11.4-ros2-$1 \
    /bin/bash -c "cd /root/ros2_ws && \
            source /opt/ros/foxy/setup.bash && \
            colcon build --symlink-install --allow-overriding cv_bridge --cmake-args -DCMAKE_BUILD_TYPE=Release"