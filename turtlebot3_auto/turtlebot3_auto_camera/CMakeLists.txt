cmake_minimum_required(VERSION 2.8.3)
project(turtlebot3_auto_camera)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

## Since OpenCV error occurred, I added this here (changed in 11.27.2017)
find_package(OpenCV REQUIRED)

catkin_python_setup()

catkin_package()

add_executable(image_to_ground src/image_to_ground.cpp)
target_link_libraries(image_to_ground
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
add_dependencies(image_to_ground turtlebot3_auto_camera_generate_messages_cpp)

include_directories(${catkin_INCLUDE_DIRS})
