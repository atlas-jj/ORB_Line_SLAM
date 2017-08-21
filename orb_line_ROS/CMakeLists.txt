cmake_minimum_required(VERSION 2.8.3)

project(orb_line)

find_package(catkin REQUIRED COMPONENTS roscpp cv_bridge geometry_msgs std_msgs image_transport)
find_package(OpenCV 3.2 REQUIRED)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES chris_test1
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)
#include_directories(${OpenCV_INCLUDE_DIRS})
 add_executable(orb_line main.cpp)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(orb_line
 ${catkin_LIBRARIES}
 ${OpenCV_LIBS}
 )

