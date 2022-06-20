cmake_minimum_required(VERSION 3.0.2)
project(sarg_2dnav)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
include ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(san_local_planner src/san_local_planner.cpp)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(follow src/follow.cpp)
add_executable(san src/san.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(follow
  ${catkin_LIBRARIES}
)
target_link_libraries(san
  ${catkin_LIBRARIES}
)
