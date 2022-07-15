cmake_minimum_required (VERSION 3.0.2)
project (microphone_direction)

find_package (catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package ()

include_directories (include ${catkin_INCLUDE_DIRS})

add_executable (directionTest src/directionTest.cpp)
add_executable (turnToSound src/turnToSound.cpp)

target_link_libraries (directionTest ${catkin_LIBRARIES})
target_link_libraries (turnToSound ${catkin_LIBRARIES})
