cmake_minimum_required(VERSION 3.0)
project(hunters_localization)

SET(CMAKE_BUILD_TYPE "Release")
add_compile_options(-std=c++11)
add_compile_options(-o3)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pcl_ros
  geometry_msgs
  tf
  eigen_conversions
)

find_package(PCL REQUIRED)

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${PCL_INCLUDE_DIRS})

catkin_package()

add_executable(front_end_node src/front_end_node.cpp)
target_link_libraries(front_end_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})