#!/bin/bash
script_dir=$(cd $(dirname $0);pwd)
sudo docker run --rm --runtime nvidia -it \
		--privileged \
		--network host \
		-e DISPLAY=$DISPLAY \
		-v /usr/bin/qemu-aarch64-static:/usr/bin/qemu-aarch64-static \
		-v /dev:/dev \
		-v /tmp/.X11-unix/:/tmp/.X11-unix \
		-v /tmp/argus_socket:/tmp/argus_socket \
		-v /etc/enctune.conf:/etc/enctune.conf \
		-v $script_dir/../../:/root/ros2_ws/src/ \
        jasonxxxyyy/sky-explorer:runtime-cuda11.4-ros2-arm64 \
        /root/ros2_ws/src/docker/scripts/vins_demo.py --pack