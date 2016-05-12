cmake_minimum_required(VERSION 2.8.3)
project(blue_robotics_t200)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

set(CMAKE_MODULE_PATH "${CMAKE_HOME_DIRECTORY}/../cmake_modules" {CMAKE_MODULE_PATH} )
find_package(catkin REQUIRED)
find_package(UCFUtilities)

## Declare external dependencies used by packages making use of this package.
catkin_package(
  INCLUDE_DIRS include ${UCFUTILITIES_INCLUDE_DIRS}
  LIBRARIES t200_thruster ${UCFUTILITIES_LIBS}
)

include_directories(include ${UCFUTILITIES_INCLUDE_DIRS})

add_library(t200_thruster src/t200_thruster.cpp)
target_link_libraries(t200_thruster ${UCFUTILITIES_LIBS})

## Install libraries and runtime dependencies
install(TARGETS t200_thruster  
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

