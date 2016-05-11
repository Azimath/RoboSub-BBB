cmake_minimum_required(VERSION 2.8.3)
project(sub_thruster_manager)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  blue_robotics_t200
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(sub_thruster_manager_node src/sub_thruster_manager_node.cpp)
target_link_libraries(sub_thruster_manager_node ${catkin_LIBRARIES})

