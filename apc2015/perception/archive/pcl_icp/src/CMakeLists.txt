cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(pcl_icp)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (convert2pcl convert2pcl.cpp)
target_link_libraries (convert2pcl ${PCL_LIBRARIES})

