cmake_minimum_required(VERSION 3.0.2)
project(debug_visualization)

add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sgtdv_msgs
  std_msgs
  visualization_msgs
  geometry_msgs
)

catkin_package(
    CATKIN_DEPENDS roscpp
    CATKIN_DEPENDS std_msgs
    CATKIN_DEPENDS sgtdv_msgs
    CATKIN_DEPENDS visualization_msgs
    CATKIN_DEPENDS geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/debug_visualization_node.cpp src/debug_visualization.cpp)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
