cmake_minimum_required (VERSION 3.0.2)
project (joint_controller)

find_package (catkin REQUIRED COMPONENTS
roscpp
actionlib
control_msgs
)

catkin_package()

include_directories (include ${catkin_INCLUDE_DIRS})

add_executable (controller_test src/controller_test.cpp)

target_link_libraries (controller_test ${catkin_LIBRARIES})
