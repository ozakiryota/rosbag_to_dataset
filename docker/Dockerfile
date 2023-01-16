########## Pull ##########
FROM ros:noetic
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
RUN mkdir -p /home/user/catkin_ws/src && \
	cd /home/user/catkin_ws && \
	/bin/bash -c "source /opt/ros/noetic/setup.bash; catkin_make" && \
	echo "source /opt/ros/noetic/setup.bash" >> /home/user/.bashrc && \
	echo "source ~/catkin_ws/devel/setup.bash" >> /home/user/.bashrc && \
	echo "export ROS_WORKSPACE=~/catkin_ws" >> /home/user/.bashrc
## cmk
RUN echo 'function cmk(){ \n\
		lastpwd=$OLDPWD \n\
		cpath=$(pwd) \n\
		cd ~/catkin_ws \n\
		catkin_make $@ \n\
		cd $cpath \n\
		OLDPWD=$lastpwd \n\
        }' >> ~/.bashrc
########## Cache busting ##########
ARG CACHEBUST=1
########## rosbag_to_dataset ##########
RUN apt-get update && \
    apt-get install -y \
		libpcl-dev \
		ros-noetic-pcl-conversions \
		ros-noetic-cv-bridge \
		ros-noetic-rviz \
		nlohmann-json3-dev && \
	cd /home/user/catkin_ws/src && \
	git clone https://github.com/ozakiryota/rosbag_to_dataset.git && \
	cd /home/user/catkin_ws && \
	/bin/bash -c "source /opt/ros/noetic/setup.bash; catkin_make"
########## User ##########
RUN apt-get update && \
	apt-get install -y gosu
COPY ./entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
########## Initial position ##########
WORKDIR /home/user/catkin_ws/src/rosbag_to_dataset
CMD ["bash"]