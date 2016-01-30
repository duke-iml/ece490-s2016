
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

set( CMAKE_RUNTIME_OUTPUT_DIRECTORY bin )

SET(KLAMPT_ROOT "/home/motion/Klampt" CACHE PATH "Klamp't path")
SET(SSPP_ROOT "${CMAKE_SOURCE_DIR}/../CoreServices"  CACHE PATH "Path to iml-internal/CoreServices")
SET (CMAKE_MODULE_PATH "${KLAMPT_ROOT}/CMakeModules")
FIND_PACKAGE(Klampt REQUIRED)
ADD_DEFINITIONS(${KLAMPT_DEFINITIONS})
INCLUDE_DIRECTORIES(${KLAMPT_INCLUDE_DIRS} ${SSPP_ROOT}/include)

#ebolabot library: real robot
SET(ROS_VERSION hydro CACHE STRING "ROS version")
IF(ROS_VERSION STREQUAL "")
ELSE(ROS_VERSION STREQUAL "")
SET(ROS_PATH /opt/ros/${ROS_VERSION})
SET(ROSDEPS roscpp roscpp_serialization rostime rosconsole)
FOREACH(NAME ${ROSDEPS})
  FIND_LIBRARY(${NAME}_LIB NAMES ${NAME} PATHS ${ROS_PATH}/lib)
  LIST(APPEND ROS_LIBRARIES ${${NAME}_LIB})
ENDFOREACH(NAME)
MESSAGE(${ROS_LIBRARIES})
SET(ROS_WS /home/motion/ros_ws CACHE STRING "ROS workspace including Baxter SDK")
INCLUDE_DIRECTORIES(${ROS_PATH}/include ${ROS_WS}/install/include ${ROS_WS}/devel/include)
ENDIF(ROS_VERSION STREQUAL "")

find_package(PCL 1.7 REQUIRED COMPONENTS common io visualization features filters segmentation search)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

ADD_EXECUTABLE(WritePCDFile src/write_pcd_file.cpp)
TARGET_LINK_LIBRARIES(WritePCDFile ${KRISLIBRARY_LIBRARIES} ${ROS_LIBRARIES} ${PCL_LIBRARIES})

ADD_EXECUTABLE(PCLWriteNormals src/pcl_write_normals.cpp)
TARGET_LINK_LIBRARIES(PCLWriteNormals ${PCL_LIBRARIES})

ADD_EXECUTABLE(Segmentation src/segmentation.cpp)
TARGET_LINK_LIBRARIES(Segmentation ${PCL_LIBRARIES})

ADD_EXECUTABLE(ICP src/icp_new.cpp)
TARGET_LINK_LIBRARIES(ICP ${PCL_LIBRARIES})

ADD_EXECUTABLE(ComplicatedRGBVisualizer src/rgb_visualizer.cpp)
TARGET_LINK_LIBRARIES(ComplicatedRGBVisualizer ${PCL_LIBRARIES})

ADD_EXECUTABLE(PLYtoPCD src/ply_to_pcd.cpp)
TARGET_LINK_LIBRARIES(PLYtoPCD ${PCL_LIBRARIES})
