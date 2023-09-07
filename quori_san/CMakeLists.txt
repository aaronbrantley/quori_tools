cmake_minimum_required (VERSION 3.0.2)
project (quori_san)

find_package (catkin REQUIRED COMPONENTS
roscpp
actionlib
tf
voting)

catkin_package ()

include_directories (include ${catkin_INCLUDE_DIRS})

add_executable (quori_san src/quori_san.cpp)

add_dependencies(quori_san ${catkin_EXPORTED_TARGETS})

target_link_libraries (quori_san ${catkin_LIBRARIES})
