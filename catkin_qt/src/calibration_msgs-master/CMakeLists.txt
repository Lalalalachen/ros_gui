cmake_minimum_required(VERSION 3.0.2)
project(calibration_msgs)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  std_msgs
  message_generation
)

# Generate messages in the 'msg' folder
 add_message_files(
   DIRECTORY msg
   FILES
   observation.msg
   image_point.msg
 )

# Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   sensor_msgs
 )

catkin_package(
  CATKIN_DEPENDS message_runtime sensor_msgs std_msgs
)
