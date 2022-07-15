cmake_minimum_required (VERSION 3.0.2)
project (base_2dnav)

find_package (catkin REQUIRED COMPONENTS
  roscpp
  actionlib
  tf
)

catkin_package ()

include_directories (include ${catkin_INCLUDE_DIRS})

add_executable (quori_wander src/quori_wander.cpp)

target_link_libraries (quori_wander ${catkin_LIBRARIES})
