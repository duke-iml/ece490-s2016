cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(icp_new)
find_package(PCL 1.3 REQUIRED COMPONENTS common io visualization)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable(icp_new icp_new.cpp)
target_link_libraries(icp_new ${PCL_LIBRARIES})

