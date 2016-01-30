CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
SET(KLAMPT_ROOT "/home/hauser/gitlibs/Klampt" CACHE PATH "Klamp't path")
SET(SSPP_ROOT "${CMAKE_SOURCE_DIR}/../CoreServices"  CACHE PATH "Path to iml-internal/CoreServices")
SET (CMAKE_MODULE_PATH "${KLAMPT_ROOT}/CMakeModules")
FIND_PACKAGE(Klampt REQUIRED) 
ADD_DEFINITIONS(${KLAMPT_DEFINITIONS})
INCLUDE_DIRECTORIES(${KLAMPT_INCLUDE_DIRS} ${SSPP_ROOT}/include)

#for SSPP (not used)
#SET(SSPP_LIBRARIES ${SSPP_ROOT}/lib/libsspp.a)

#for motion
INCLUDE_DIRECTORIES(Motion)

#ebolabot library: real robot
SET(ROS_VERSION hydro CACHE STRING "ROS version")
IF(ROS_VERSION STREQUAL "")
  SET(ROS_FOUND FALSE)
ELSE(ROS_VERSION STREQUAL "")
  SET(ROS_FOUND TRUE)
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

#ebolabot library: virtual robots
ADD_LIBRARY(motion_kinematic SHARED
		Motion/motion_kinematic.cpp)
TARGET_LINK_LIBRARIES(motion_kinematic ${SSPP_LIBRARIES} ${KLAMPT_LIBRARIES})
#ADD_LIBRARY(motion_klamptsim SHARED
#		Motion/motion_klamptsim.cpp)
#TARGET_LINK_LIBRARIES(motion_klamptsim ${SSPP_LIBRARIES}) ${KLAMPT_LIBRARIES})

IF(ROS_FOUND)
  ADD_LIBRARY(motion_physical SHARED
  Motion/motion_physical.cpp)
  TARGET_LINK_LIBRARIES(motion_physical ${ROS_LIBRARIES} ${SSPP_LIBRARIES} ${KLAMPT_LIBRARIES})

  ADD_EXECUTABLE(RealSense_ROS_Emitter Sensors/realsense_ros_emitter.cpp)
TARGET_LINK_LIBRARIES(RealSense_ROS_Emitter ${KRISLIBRARY_LIBRARIES} ${ROS_LIBRARIES})

  find_package(PCL 1.2)
  IF(PCL_FOUND)
    include_directories(${PCL_INCLUDE_DIRS})
    link_directories(${PCL_LIBRARY_DIRS})
    add_definitions(${PCL_DEFINITIONS})

    ADD_EXECUTABLE(RealSense_ROS_Emitter_PCL Sensors/realsense_ros_emitter_with_pcl.cpp)
    TARGET_LINK_LIBRARIES(RealSense_ROS_Emitter_PCL ${KRISLIBRARY_LIBRARIES} ${ROS_LIBRARIES} ${PCL_LIBRARIES})
  ENDIF(PCL_FOUND)
ENDIF(ROS_FOUND)



