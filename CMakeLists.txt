cmake_minimum_required(VERSION 2.8.3)
project(plane_localization)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  rospy
  cv_bridge
  # pcl library
  pcl_conversions
  # msgs
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
)

find_package(PCL REQUIRED)

catkin_package(
  CATKIN_DEPENDS
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
)

# include directories
include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/livox_ros_driver
  ${CMAKE_CURRENT_SOURCE_DIR}/plane_octree
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
)

# link directories
link_directories(
	include
	${PCL_LIBRARY_DIRS}
)

###########
## Build ##
###########

# Range Image Projection
add_executable(plane_localization src/main.cpp)
target_link_libraries(plane_localization ${catkin_LIBRARIES} ${PCL_LIBRARIES})