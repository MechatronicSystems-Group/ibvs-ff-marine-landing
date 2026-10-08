docker run -it \
	--privileged \
	--network host \
	--ipc=host \
	-v /home/rpi/ws_x500:/ws_x500 \
	-v /dev:/dev \
	-v /run/udev:/run/udev \
	ros2-humble-x500
