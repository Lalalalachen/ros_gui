cmake_minimum_required(VERSION 3.0.2)
project(temporary_calib_interface)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

# Find catkin macros and libraries
# if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
# is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  calibration_msgs
  message_filters
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs calibration_msgs message_filters
)

###########
## Build ##
###########
add_executable(calib_view src/calib_view.cpp)
target_link_libraries(calib_view ${catkin_LIBRARIES})
add_dependencies(calib_view ${catkin_EXPORTED_TARGETS})

include_directories(
 ${catkin_INCLUDE_DIRS}
)

