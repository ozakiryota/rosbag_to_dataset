cmake_minimum_required(VERSION 3.0.2)
project(rosbag_to_dataset)

add_compile_options(-std=c++17 -O2 -g -Wall)

find_package(catkin REQUIRED
    roscpp
    rosbag
    cv_bridge
)
find_package(PCL)
find_package(OpenCV)

################################################
## Declare ROS messages, services and actions ##
################################################

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

catkin_package(
)

###########
## Build ##
###########

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

add_executable(rosbag_to_dataset
    src/rosbag_to_dataset.cpp
)
target_link_libraries(rosbag_to_dataset
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

#############
## Install ##
#############

#############
## Testing ##
#############
