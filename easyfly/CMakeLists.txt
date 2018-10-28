cmake_minimum_required(VERSION 2.8.3)
project(easyfly)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  crazyflie_cpp
  geometry_msgs
  message_generation
  vicon_bridge
  genmsg
  cv_bridge
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate services in the 'srv' folder
add_service_files(
  FILES
  Swarm_Add.srv
  UpdateParams.srv
)

add_message_files(
  FILES
  commands.msg
  LogBlock.msg
  GenericLogData.msg
  pos_ctrl_sp.msg
  raw_ctrl_sp.msg
  trj_ctrl_sp.msg
  pos_est.msg
  output.msg
  Recording.msg
  att_est.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages( DEPENDENCIES
   std_msgs
   geometry_msgs
)

# Enable C++11
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS 
  program_options 
#  bind
  REQUIRED)

FIND_PACKAGE(OpenCV REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES easyfly vicon_bridge
#  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs vicon_bridge
#  DEPENDS system_lib
  CATKIN_DEPENDS
  message_runtime
  std_msgs
  vicon_bridge
  crazyflie_cpp
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
./include
 # /home/wade/catkin_ws/src/crazyflie_ros-first_trails/easyfly/include
)

#add_library(gl_declair SHARED include/gl_declair.cpp)
add_library(IIR SHARED include/IIR.cpp)
add_library(type_methode SHARED include/type_methode.cpp)
#add_library(Controller_Decentrolized SHARED src/swarm_controller_Decentralized.cpp)

add_executable(commander
  src/commander.cpp
)

target_link_libraries(commander
  #GLU GL glut
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBS}
 # /home/wade/catkin_ws/src/crazyflie_ros-first_trails/easyfly/include/type_methode.cpp
 # /home/wade/catkin_ws/src/crazyflie_ros-first_trails/easyfly/include/gl_declair.cpp
 #gl_declair
 type_methode
)

add_executable(swarm_Server
  src/swarm_Server.cpp
)

target_link_libraries(swarm_Server
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
 # /home/wade/catkin_ws/src/crazyflie_ros-first_trails/easyfly/include/type_methode.cpp
 type_methode
)

add_executable(swarm_addCf
  src/swarm_addCf.cpp
)

#add_dependencies(swarm_addCf
#  crazyflie_driver_generate_messages_cpp
#)

target_link_libraries(swarm_addCf
  ${catkin_LIBRARIES}
)
add_executable(swarm_controller
  src/swarm_controller.cpp
)

target_link_libraries(swarm_controller
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
 #  /home/wade/catkin_ws/src/crazyflie_ros-first_trails/easyfly/include/type_methode.cpp
 #  /home/wade/catkin_ws/src/crazyflie_ros-first_trails/easyfly/include/IIR.cpp
 IIR
 type_methode
)

add_executable(joysticks2
  src/joysticks2.cpp
)
target_link_libraries(joysticks2
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(swarm_Server_Decentralized
  src/swarm_Server_Decentralized.cpp
)

#add_dependencies(swarm_addCf
#  crazyflie_driver_generate_messages_cpp
#)

target_link_libraries(swarm_Server_Decentralized
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  type_methode
  #Controller_Decentrolized
  IIR
)

#add_executable(xbee_linker
#  src/xbee_linker.cpp
#)
#target_link_libraries(xbee_linker
#  ${catkin_LIBRARIES}
#  ${Boost_LIBRARIES}
#)

#add_executable(xbee_linker2
#  src/xbee_linker2.cpp
#)
#target_link_libraries(xbee_linker2
#  ${catkin_LIBRARIES}
#  ${Boost_LIBRARIES}
#)

