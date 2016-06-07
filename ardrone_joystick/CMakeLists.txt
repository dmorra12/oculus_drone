cmake_minimum_required(VERSION 2.8.3)
project(ardrone_joystick)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  std_srvs
)

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(ardrone_joystick_node src/main.cpp)
add_executable(ardrone_joystick_only_node src/main_joy_only.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(ardrone_joystick_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(ardrone_joystick_node
  ${catkin_LIBRARIES}
)
target_link_libraries(ardrone_joystick_only_node
  ${catkin_LIBRARIES}
)
