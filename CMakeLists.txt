cmake_minimum_required(VERSION 3.0.2)
project(voronoi_planner)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror -Wall")

# set(CMAKE_CXX_CLANG_TIDY
#   clang-tidy;
#   -header-filter=${PROJECT_SOURCE_DIR}/voronoi_planner;
#   -warnings-as-errors=*;
# )

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  geometry_msgs
  nav_core
  nav_msgs
  pluginlib
  voronoi_layer
)

find_package(glog REQUIRED)

catkin_package(
  INCLUDE_DIRS ${PROJECT_SOURCE_DIR}
  LIBRARIES voronoi_planner
  CATKIN_DEPENDS
    costmap_2d
    geometry_msgs
    nav_core
    nav_msgs
    pluginlib
    voronoi_layer
)

include_directories(
  ${PROJECT_SOURCE_DIR}
  ${catkin_INCLUDE_DIRS}
)

add_library(voronoi_planner
  voronoi_planner/common/heap.cc
  voronoi_planner/voronoi_planner_ros.cc
  voronoi_planner/voronoi_planner.cc
)
target_link_libraries(voronoi_planner ${catkin_LIBRARIES} glog::glog)
