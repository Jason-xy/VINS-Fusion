sudo docker run --runtime nvidia -it --rm \
		--network host \
		-e DISPLAY=$DISPLAY \
		-v /tmp/.X11-unix/:/tmp/.X11-unix \
		-v /tmp/argus_socket:/tmp/argus_socket \
		-v /etc/enctune.conf:/etc/enctune.conf \
        jasonxxxyyy/sky-explorer:vins-fusion-ros2 \
        /bin/bash