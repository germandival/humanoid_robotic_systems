cmake_minimum_required(VERSION 2.8.3)
project(planner)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  rospy
  message_generation
  sensor_msgs
  cv_bridge
  image_transport
)
find_package(OpenCV REQUIRED)

add_service_files(
    DIRECTORY srv
    FILES
    ChangePosture.srv
    DistanceCamFoot.srv
    PosArucoFloor.srv
    MoveTo.srv
)

generate_messages(DEPENDENCIES std_msgs)
catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS ./script/change_posture_service.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS ./script/move_to_service.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
) 
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
add_executable(planner src/main.cpp)
target_link_libraries(planner ${catkin_LIBRARIES})

find_package(OpenCV REQUIRED)
target_link_libraries(planner ${OpenCV_LIBS})
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} /usr/local/lib/cmake)
find_package(aruco REQUIRED)
target_link_libraries(planner ${aruco_LIBS})
