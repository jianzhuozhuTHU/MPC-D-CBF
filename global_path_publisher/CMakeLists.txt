cmake_minimum_required(VERSION 3.0.2)
project(global_path_publisher)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES global_path_publisher
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

add_executable(global_path_pub src/global_path_publisher.cpp)

target_link_libraries(global_path_pub
    ${catkin_LIBRARIES}
)

