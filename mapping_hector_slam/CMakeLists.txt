cmake_minimum_required(VERSION 2.8.3)
project(mapping_hector_slam)

find_package(catkin REQUIRED COMPONENTS
  hector_mapping
  hsrb_gazebo_launch
  tmc_potential_maps
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mapping_hector_slam
#  CATKIN_DEPENDS hector_mapping hsrb_gazebo_launch tmc_potential_maps
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

