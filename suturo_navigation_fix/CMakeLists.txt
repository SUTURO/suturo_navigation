cmake_minimum_required(VERSION 2.8.3)
project(suturo_navigation_fix)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  move_base_msgs
  rospy
  actionlib
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES snap_map_icp-launch
#  CATKIN_DEPENDS snap_map_icp
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


