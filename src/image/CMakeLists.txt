cmake_minimum_required(VERSION 2.8.3)
project(image)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  #sensor_msgs
  cv_bridge
  roscpp
  std_msgs
  image_transport
  tf
)

catkin_package(
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(image_reader src/image_reader.cpp)
target_link_libraries(image_reader ${catkin_LIBRARIES})

add_executable(image_reader_programming src/image_reader_programming.cpp)
target_link_libraries(image_reader_programming ${catkin_LIBRARIES})
