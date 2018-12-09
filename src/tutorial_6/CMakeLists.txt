cmake_minimum_required(VERSION 2.8.3)
project(tutorial_6)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  rospy
  message_generation
  sensor_msgs
  cv_bridge
  image_transport
)

add_service_files(
    DIRECTORY srv
    FILES
    RoiFace.srv
    NaoFace.srv
)

generate_messages(DEPENDENCIES std_msgs)
catkin_package(CATKIN_DEPENDS message_runtime)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS ./script/move_service.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
add_executable(nao_1 src/main.cpp)
target_link_libraries(nao_1 ${catkin_LIBRARIES})

find_package(OpenCV REQUIRED)
target_link_libraries(nao_1 ${OpenCV_LIBS})
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} /usr/local/lib/cmake)
find_package(aruco REQUIRED)
target_link_libraries(nao_1  ${aruco_LIBS})
