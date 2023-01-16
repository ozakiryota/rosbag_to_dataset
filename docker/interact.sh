#!/bin/bash

xhost +

image="rosbag_to_dataset"
tag="latest"
home_dir="/home/user"

docker run \
	-it \
	--rm \
	-e local_uid=$(id -u $USER) \
	-e local_gid=$(id -g $USER) \
	-e "DISPLAY" \
	-v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
	-v $(pwd)/..:$home_dir/catkin_ws/src/$image \
	-v $HOME/rosbag:$home_dir/rosbag \
	$image:$tag