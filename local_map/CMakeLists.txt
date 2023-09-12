cmake_minimum_required( VERSION 2.8 )
project(local_map)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS 
    roscpp 
    std_msgs
    rviz_visual_tools
    geometry_msgs
    grid_map_octomap
    grid_map_ros
    sensor_msgs
    pcl_conversions
    tf_conversions
)

find_package(PCL REQUIRED)
find_package(CGAL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
    include 
    ${catkin_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
)

add_executable(local_map_pub src/local_map_pub.cpp)

target_link_libraries(local_map_pub 
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES} 
    ${CGAL_LIB}
    ${GMP_LIBRARIES}
)