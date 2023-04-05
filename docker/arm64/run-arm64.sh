#!/bin/bash
xhost +
script_dir=$(cd $(dirname $0);pwd)
docker run -it \
		--privileged \
		--network host \
		-e DISPLAY=$DISPLAY \
		-v /dev:/dev \
		-v /tmp/.X11-unix/:/tmp/.X11-unix \
		-v /tmp/argus_socket:/tmp/argus_socket \
		-v /etc/enctune.conf:/etc/enctune.conf \
		-v $script_dir/../../:/root/ros2_ws/src/ \
        jasonxxxyyy/sky-explorer:runtime-cuda11.4-ros2-arm64 \
		/root/ros2_ws/src/docker/scripts/vins_demo.py --rviz