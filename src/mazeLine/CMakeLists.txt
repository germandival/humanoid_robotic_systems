cmake_minimum_required(VERSION 2.8.3)
project(mazeLine)

set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_MODULE_PATH /usr/local/lib/cmake)

find_package(catkin REQUIRED cmake_modules COMPONENTS
  roscpp
  std_msgs
  naoqi_bridge_msgs
  std_srvs
  tf
  message_generation
  actionlib_msgs
  geometry_msgs
  cv_bridge
  sensor_msgs
  visualization_msgs
)

find_package(OpenCV REQUIRED)
find_package(aruco REQUIRED)

add_service_files(
    DIRECTORY srv
    FILES
    next_step.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package()

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${aruco_INCLUDE_DIRS}
)

add_executable(mazeLine src/main.cpp)

target_link_libraries(mazeLine ${catkin_LIBRARIES} ${catkin_LIBRARIES} ${roscpp_LIBRARIES} ${OpenCV_LIBRARIES} ${aruco_LIBS})

find_package(Boost REQUIRED COMPONENTS system thread)
include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(mazeLine ${Boost_LIBRARIES})
