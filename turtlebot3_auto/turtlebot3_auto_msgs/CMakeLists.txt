cmake_minimum_required(VERSION 2.8.3)
project(turtlebot3_auto_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  dynamic_reconfigure
)

add_message_files(
  FILES
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS std_msgs message_runtime
)
