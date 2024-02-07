cmake_minimum_required(VERSION 3.0.2)
project(ur5)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  message_generation
  roslaunch
)

find_package(Eigen3 REQUIRED)

roslaunch_add_file_check(launch)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  ServiceMessage.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy std_msgs ur5_motion_library ur5_task_library
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(
  ur5_motion_library 
  src/ur5_motion_library.cpp
)

add_library(
  ur5_task_library
  src/ur5_task_library.cpp
)

target_link_libraries(ur5_motion_library ${catkin_LIBRARIES} Eigen3::Eigen)
target_link_libraries(ur5_task_library ${catkin_LIBRARIES} Eigen3::Eigen)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/ur5_node.cpp)

add_executable(motion_planner src/motion_planner.cpp)
add_executable(task_planner src/task_planner.cpp)

target_link_libraries(motion_planner
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  ur5_motion_library
)

target_link_libraries(task_planner
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  ur5_task_library
)