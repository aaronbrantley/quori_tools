cmake_minimum_required (VERSION 3.0.2)
project (microphone_direction)

find_package (catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package ()

include_directories (include ${catkin_INCLUDE_DIRS})

add_executable (direction_test src/direction_test.cpp)
add_executable (turn_to_sound src/turn_to_sound.cpp)

target_link_libraries (direction_test ${catkin_LIBRARIES})
target_link_libraries (turn_to_sound ${catkin_LIBRARIES})
