########## Pull ##########
FROM ros:noetic
########## User ##########
ARG home_dir="/home/user"
COPY ./entrypoint.sh /usr/local/bin/entrypoint.sh
RUN apt-get update && \
	apt-get install -y \
		gosu \
		sudo && \
	chmod +x /usr/local/bin/entrypoint.sh && \
	mkdir -p $home_dir
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
########## Non-interactive ##########
ENV DEBIAN_FRONTEND=noninteractive
########## Common tool ##########
RUN apt-get update && \
	apt-get install -y \
		vim \
		wget \
		unzip \
		git \
        python-tk
########## ROS setup ##########
RUN mkdir -p $home_dir/catkin_ws/src && \
	cd $home_dir/catkin_ws && \
	/bin/bash -c "source /opt/ros/noetic/setup.bash; catkin_make" && \
	echo "source /opt/ros/noetic/setup.bash" >> $home_dir/.bashrc && \
	echo "source ~/catkin_ws/devel/setup.bash" >> $home_dir/.bashrc && \
	echo "export ROS_WORKSPACE=~/catkin_ws" >> $home_dir/.bashrc
## cmk
RUN echo 'function cmk(){ \n\
		lastpwd=$OLDPWD \n\
		cpath=$(pwd) \n\
		cd ~/catkin_ws \n\
		catkin_make $@ \n\
		cd $cpath \n\
		OLDPWD=$lastpwd \n\
        }' >> $home_dir/.bashrc
########## Cache busting ##########
ARG cache_bust=1
########## rosbag_to_dataset ##########
RUN apt-get update && \
    apt-get install -y \
		libpcl-dev \
		ros-noetic-pcl-conversions \
		ros-noetic-cv-bridge \
		ros-noetic-rviz \
		nlohmann-json3-dev && \
	cd $home_dir/catkin_ws/src && \
	git clone https://github.com/ozakiryota/rosbag_to_dataset.git && \
	cd $home_dir/catkin_ws && \
	/bin/bash -c "source /opt/ros/noetic/setup.bash; catkin_make"
########## Initial position ##########
WORKDIR $home_dir/catkin_ws/src/rosbag_to_dataset
CMD ["bash"]