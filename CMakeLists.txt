cmake_minimum_required(VERSION 2.8.3)
project(nav_goal)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  move_base
  nav_msgs
  roscpp
  std_msgs
  tf
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES nav_goal
#  CATKIN_DEPENDS geometry_msgs move_base nav_msgs roscpp std_msgs tf
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/goal.cpp)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

