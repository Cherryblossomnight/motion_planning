cmake_minimum_required(VERSION 2.8.3)
project(push_box)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  squeeze_navigation
  )

catkin_package(
  INCLUDE_DIRS include
  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)


add_library(push_box src/push_box.cpp)
target_link_libraries(push_box ${catkin_LIBRARIES})
add_dependencies(push_box ${catkin_EXPORTED_TARGETS})

add_executable(push_box_node src/push_box_node.cpp)
target_link_libraries(push_box_node push_box ${catkin_LIBRARIES})
