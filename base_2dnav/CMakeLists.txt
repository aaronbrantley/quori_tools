cmake_minimum_required(VERSION 3.0.2)
project(base_2dnav)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib
  tf
)

catkin_package ()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(quori_wander src/quori_wander.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(quori_wander ${catkin_LIBRARIES})
