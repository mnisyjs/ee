cmake_minimum_required(VERSION 3.0.2)
project(arm_control_node)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  eyes2hand
  tf
  moveit_ctrl
  piper_msgs
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  CameraTargets.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS 
    roscpp 
    std_msgs 
    geometry_msgs 
    nav_msgs 
    eyes2hand
    tf 
    moveit_ctrl 
    piper_msgs
    message_runtime
)

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(arm_control_node arm_control_node.cpp)

## Add cmake target dependencies of the executable
add_dependencies(arm_control_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(arm_control_node
  ${catkin_LIBRARIES}
)
