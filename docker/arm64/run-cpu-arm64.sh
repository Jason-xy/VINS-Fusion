#!/bin/bash
script_dir=$(cd $(dirname $0);pwd)
docker run --rm -it \
        --name vins \
        --privileged \
        --network host \
        -e DISPLAY=$DISPLAY \
        -v /dev:/dev \
        -v /tmp/.X11-unix/:/tmp/.X11-unix \
        -v /tmp/argus_socket:/tmp/argus_socket \
        -v /etc/enctune.conf:/etc/enctune.conf \
        -v $script_dir/../../../../:/root/ros2_ws/ \
        jasonxxxyyy/sky-explorer:runtime-cpu-ros2-arm64 \
        /root/ros2_ws/src/VINS-Fusion/docker/scripts/vins_demo.py --demo