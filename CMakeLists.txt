cmake_minimum_required(VERSION 3.0.2)
project(voronoi_planner)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  geometry_msgs
  nav_core
  nav_msgs
  pluginlib
  roscpp
  voronoi_layer
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES voronoi_planner
  CATKIN_DEPENDS costmap_2d geometry_msgs nav_core nav_msgs pluginlib roscpp voronoi_layer
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/heap.cpp
  src/voronoi_planner_ros.cpp
  src/voronoi_planner.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
