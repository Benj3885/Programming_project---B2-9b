cmake_minimum_required(VERSION 2.8.3)
project(brain)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  actionlib
  nav_msgs
  move_base_msgs
  tf
)

catkin_package(
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(center src/center.cpp)
target_link_libraries(center ${catkin_LIBRARIES})

add_executable(center_programming src/center_programming.cpp)
target_link_libraries(center_programming ${catkin_LIBRARIES})
