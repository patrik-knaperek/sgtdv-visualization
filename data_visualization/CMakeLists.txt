cmake_minimum_required(VERSION 3.0.2)
project(data_visualization)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sgtdv_msgs
  std_msgs
)

find_package(Eigen3 REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package()

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/data_visualization.cpp src/data_visualization_node.cpp)

target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
)
