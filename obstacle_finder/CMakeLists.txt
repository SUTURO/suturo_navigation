cmake_minimum_required(VERSION 2.8.3)
project(obstacle_finder)
find_package(catkin REQUIRED
    COMPONENTS
    sensor_msgs
    nav_msgs
    geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package()