cmake_minimum_required(VERSION 3.0.2)
project(obs_param)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  visualization_msgs 
)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(obs_kf src/obs_kf.cpp src/kalman.cpp)

target_link_libraries(obs_kf
  ${catkin_LIBRARIES}
)


