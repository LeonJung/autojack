cmake_minimum_required(VERSION 2.8.3)
project(turtlebot3_auto_detect)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  dynamic_reconfigure
)

## Since OpenCV error occurred, I added this here (changed in 11.27.2017)
find_package(OpenCV REQUIRED)

catkin_python_setup()

generate_dynamic_reconfigure_options(
  cfg/DetectLaneParams.cfg
)

catkin_package(
  CATKIN_DEPENDS dynamic_reconfigure rospy std_msgs
)

include_directories(${catkin_INCLUDE_DIRS})



