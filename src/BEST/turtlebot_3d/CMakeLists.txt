cmake_minimum_required(VERSION 2.8.3)
project(turtlebot_3d)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS rospy)


## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(monitor_3d src/monitor_3d.py)
target_link_libraries(monitor ${catkin_LIBRARIES})
add_dependencies(monitor_3d turtlebot_3d_generate_messages_py)
