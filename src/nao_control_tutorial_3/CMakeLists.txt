cmake_minimum_required(VERSION 2.8.3)
project(nao_control_tutorial_3)
SET(CMAKE_MODULE_PATH ${CMAKE_INSTALL_PREFIX}/lib/cmake/ )
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_MODULE_PATH /usr/local/lib/cmake)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
## find_package(catkin REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


find_package(catkin REQUIRED cmake_modules COMPONENTS
  roscpp
  rospy
  std_msgs
  naoqi_bridge_msgs
  std_srvs
  tf
  actionlib_msgs
  geometry_msgs
  sensor_msgs
  cv_bridge
  image_transport
  message_generation
)



find_package(OpenCV)
find_package(aruco REQUIRED)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)
#SET(CMAKE_MODULE_PATH ${CMAKE_INSTALL_PREFIX}/lib/cmake/ )

add_service_files(
    DIRECTORY srv
    FILES
    TrackBall.srv
    GrabBall.srv
    HandOverBall.srv
)

generate_messages(DEPENDENCIES std_msgs)
catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS ./script/track_service.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS ./script/grab_ball.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS ./script/handover_ball.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


find_package(Boost REQUIRED COMPONENTS system thread)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(include ${aruco_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})

add_executable(nao_3 src/main.cpp)
target_link_libraries(nao_3 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${aruco_LIBS} ${Boost_LIBRARIES})
