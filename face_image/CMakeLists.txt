cmake_minimum_required (VERSION 3.0.2)
project (face_image)

find_package (catkin REQUIRED COMPONENTS
  roscpp
  roslib
  image_transport
  cv_bridge
)

catkin_package()

include_directories (include ${catkin_INCLUDE_DIRS})

add_executable (faceTest src/faceTest.cpp)
add_executable (expressions src/expressions.cpp)

target_link_libraries (faceTest ${catkin_LIBRARIES})
target_link_libraries (expressions ${catkin_LIBRARIES})
