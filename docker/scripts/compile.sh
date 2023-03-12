#!/bin/bash
script_dir=$(cd $(dirname $0);pwd)
file=/usr/bin/qemu-aarch64-static
if [ ! -f "$file" ]; then
  wget https://github.com/multiarch/qemu-user-static/releases/download/v7.2.0-1/qemu-aarch64-static -o /usr/bin/qemu-aarch64-static
fi

docker run --rm \
        --network host \
        --privileged \
        -v /usr/bin/qemu-aarch64-static:/usr/bin/qemu-aarch64-static \
        -v $script_dir/../../:/root/ros2_ws/src/ \
        jasonxxxyyy/sky-explorer:runtime-cuda11.4-ros2-arm64 \
        /root/ros2_ws/src/docker/scripts/vins_demo.py --pack