cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(RealSensePCD)
find_package(PCL 1.3 REQUIRED COMPONENTS)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable(realsense_ros_emitter_with_pcl realsense_ros_emitter_with_pcl.cpp)
target_link_libraries(realsense_ros_emitter_with_pcl ${KRISLIBRARY_LIBRARIES} ${ROS_LIBRARIES} ${PCL_LIBRARIES})


