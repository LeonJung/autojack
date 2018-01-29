cmake_minimum_required(VERSION 2.8.3)
project(turtlebot3_auto_camera)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  dynamic_reconfigure
  OpenCV
)

catkin_python_setup()

generate_dynamic_reconfigure_options(
  cfg/ImageCompensationParams.cfg
)

# make sure configure headers are built before any node using them
#add_dependencies(image_compensation ${PROJECT_NAME}_gencfg)

catkin_package(
  CATKIN_DEPENDS dynamic_reconfigure rospy std_msgs
)

include_directories(${catkin_INCLUDE_DIRS})
