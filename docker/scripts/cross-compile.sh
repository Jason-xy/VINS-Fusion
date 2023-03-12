#!/bin/bash
script_dir=$(cd $(dirname $0);pwd)
docker run --rm --privileged multiarch/qemu-user-static:register --reset
docker run --rm \
        --network host \
        -v /usr/bin/qemu-aarch64-static:/usr/bin/qemu-aarch64-static \
        -v $script_dir/../../:/root/ros2_ws/src/ \
        jasonxxxyyy/sky-explorer:runtime-cuda11.4-ros2-arm64 \
        /root/ros2_ws/src/docker/scripts/vins_demo.py --pack