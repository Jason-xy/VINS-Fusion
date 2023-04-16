#!/bin/bash
xhost +
script_dir=$(cd $(dirname $0);pwd)
docker run --gpus all --rm -it \
        --name vins \
        --privileged \
        --network host \
        -e DISPLAY=$DISPLAY \
        -v /tmp/.X11-unix/:/tmp/.X11-unix \
        -v $script_dir/../../../../:/root/ros2_ws/ \
        jasonxxxyyy/sky-explorer:runtime-cuda11.4-ros2-amd64 \
       	/root/ros2_ws/src/VINS-Fusion/docker/scripts/vins_demo.py --rviz