#!/bin/bash
docker run --gpus all -it --rm \
		--name vins \
		--privileged \
		--network host \
		-e DISPLAY=$DISPLAY \
		-v /dev:/dev \
		-v /tmp/.X11-unix/:/tmp/.X11-unix \
		-v /tmp/argus_socket:/tmp/argus_socket \
		-v /etc/enctune.conf:/etc/enctune.conf \
        jasonxxxyyy/sky-explorer:runtime-cuda11.4-ros2-amd64 \
        /bin/bash