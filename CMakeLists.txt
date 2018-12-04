cmake_minimum_required(VERSION 2.8.3)
project(rosie_linear_navigator)

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs rosie_path_finder sensor_msgs nav_msgs tf visualization_msgs)

catkin_package(
  DEPENDS
  CATKIN_DEPENDS roscpp rosie_path_finder geometry_msgs sensor_msgs nav_msgs tf visualization_msgs
  LIBRARIES
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(rosie_linear_navigator src/linear_navigator_node.cpp)
target_link_libraries(rosie_linear_navigator ${catkin_LIBRARIES})
add_dependencies(rosie_linear_navigator rosie_path_finder ${catkin_EXPORTED_TARGETS})
