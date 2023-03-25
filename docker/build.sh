#!/bin/bash

image="rosbag_to_dataset"
tag="latest"
exec_pwd=$(cd $(dirname $0); pwd)

docker build $exec_pwd \
    -t $image:$tag \
    --build-arg cache_bust=$(date +%s)