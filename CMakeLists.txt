cmake_minimum_required(VERSION 2.8.3)
project(icp_map_combiner)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
)
find_package(Eigen3 REQUIRED)

add_definitions(-std=c++11)
add_definitions(${EIGEN_DEFINITIONS})

catkin_package(
  #INCLUDE_DIRS include
  LIBRARIES icp_map_combiner
  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp
  DEPENDS eigen
)

add_executable(icp_map_combiner src/main.cpp)

target_link_libraries(icp_map_combiner
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
