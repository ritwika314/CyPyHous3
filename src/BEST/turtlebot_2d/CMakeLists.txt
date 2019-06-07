cmake_minimum_required(VERSION 2.8.3)
project(turtlebot_2d)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp turtlesim geometry_msgs std_srvs)


## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(monitor src/monitor.cpp)
target_link_libraries(monitor ${catkin_LIBRARIES})
add_dependencies(monitor turtlebot_2d_generate_messages_cpp)
