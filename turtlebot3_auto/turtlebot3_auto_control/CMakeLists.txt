cmake_minimum_required(VERSION 2.8.3)
project(turtlebot3_auto_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

## Since OpenCV error occurred, I added this here (changed in 11.27.2017)
find_package(OpenCV REQUIRED)

catkin_python_setup()

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
