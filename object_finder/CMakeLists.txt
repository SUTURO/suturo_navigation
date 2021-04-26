cmake_minimum_required(VERSION 2.8.3)
project(object_finder)

find_package(catkin REQUIRED
    COMPONENTS
    roscpp
    costmap_2d
    dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
    cfg/ObjectFinder.cfg
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp costmap_2d dynamic_reconfigure
)

include_directories(
  ${catkin_INCLUDE_DIRS} include
)

add_library(static_clearing_layer src/static_clearing_layer.cpp)
add_dependencies(static_clearing_layer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(static_clearing_layer ${catkin_LIBRARIES})

add_executable(object_finder src/object_finder_node.cpp)
add_dependencies(object_finder ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
target_link_libraries(object_finder ${catkin_LIBRARIES})# object_finder)

install( TARGETS
    object_finder
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS
    static_clearing_layer
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(FILES costmap_plugins.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)