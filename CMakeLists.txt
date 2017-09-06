cmake_minimum_required(VERSION 2.8.3)
project(stitch_two)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include/${PROJECT_NAME}
)

add_executable(stitch 
   src/stitch.cpp src/VideoLoader.cpp
)

target_link_libraries(stitch ${catkin_LIBRARIES} ${OpenCV_LIBS})
