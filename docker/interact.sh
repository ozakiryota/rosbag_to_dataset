#!/bin/bash

xhost +

image="rosbag_to_dataset"
tag="latest"

docker run \
	-it \
	--rm \
	-e "DISPLAY" \
	-v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
	-v $(pwd)/..:/root/catkin_ws/src/$image \
	-v $HOME/rosbag:/root/rosbag \
	$image:$tag