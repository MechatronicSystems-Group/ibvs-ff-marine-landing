docker run -it \
	--privileged \
	--name ros2-x500 \
	--network host \
	--ipc=host \
	-v /home/rpi/ws_x500:/home/docker/ws_x500 \
	-v /dev:/dev \
	-v /run/udev:/run/udev \
	ros2-humble-x500
