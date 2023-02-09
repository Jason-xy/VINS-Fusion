#!/bin/bash
script_dir=$(cd $(dirname $0);pwd)
docker build -t jasonxxxyyy/sky-explorer:vins-fusion-amd64 -f $script_dir/amd64.dockerfile $script_dir/../../
docker run --gpus all -it --rm \
		--name vins \
		--network host \
		-e DISPLAY=$DISPLAY \
		-v /tmp/.X11-unix/:/tmp/.X11-unix \
        jasonxxxyyy/sky-explorer:vins-fusion-amd64 \
       	/root/ros2_ws/src/docker/scripts/vins_demo.py