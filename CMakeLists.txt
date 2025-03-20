cmake_minimum_required(VERSION 3.0.2)
project(soccer_gazebo_sim)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  robotis_controller_msgs
  op3_action_module_msgs
  op3_walking_module_msgs
  op3_online_walking_module_msgs
  op3_tuning_module_msgs
  cmake_modules
  dynamixel_sdk
  robotis_framework_common
  robotis_device
  robotis_controller
  robotis_math
  op3_kinematics_dynamics
  op3_balance_control
  open_cr_module
  op3_base_module
  op3_head_control_module
  op3_walking_module
  op3_action_module
  op3_direct_control_module
  op3_online_walking_module
  op3_tuning_module
  sensor_msgs
  cv_bridge
  image_transport
  geometry_msgs
)


catkin_package(
  CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    robotis_controller_msgs
    op3_action_module_msgs
    op3_walking_module_msgs
    op3_online_walking_module_msgs
    op3_tuning_module_msgs
    cmake_modules
    dynamixel_sdk
    robotis_framework_common
    robotis_device
    robotis_controller
    robotis_math
    op3_kinematics_dynamics
    op3_balance_control
    open_cr_module
    op3_base_module
    op3_head_control_module
    op3_walking_module
    op3_action_module
    op3_direct_control_module
    op3_online_walking_module
    op3_tuning_module
    sensor_msgs
    cv_bridge
    image_transport
    geometry_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/soccer_gazebo_sim.cpp
# )


## Declare a C++ executable
#add_executable(${PROJECT_NAME}_node src/soccer_pkg_node.cpp)
add_executable(walking_gazebo
  src/walking_gazebo.cpp
)

## Add cmake target dependencies of the library
#add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# add_dependencies(soccer_gazebo_sim
#   ${${PROJECT_NAME}_EXPORTED_TARGETS}
#   ${catkin_EXPORTED_TARGETS}
# )

## Specify libraries to link a library or executable target against
target_link_libraries(walking_gazebo
  ${catkin_LIBRARIES}
  ${Eigen3_LIBRARIES}
)
