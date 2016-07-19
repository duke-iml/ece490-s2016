CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
SET(KLAMPT_ROOT "/home/motion/Klampt" CACHE PATH "Klamp't path")
SET(SSPP_ROOT "${CMAKE_SOURCE_DIR}/SSPP"  CACHE PATH "Path to SSPP")
SET (CMAKE_MODULE_PATH "${KLAMPT_ROOT}/CMakeModules")
FIND_PACKAGE(Klampt REQUIRED) 
ADD_DEFINITIONS(${KLAMPT_DEFINITIONS})
INCLUDE_DIRECTORIES(. ${KLAMPT_INCLUDE_DIRS} ${SSPP_ROOT}/include)

#for SSPP
SET(SSPP_LIBRARIES ${SSPP_ROOT}/lib/libsspp.a)

#for ebolabot
INCLUDE_DIRECTORIES(Motion)

#necessary for Python task controllers
find_package ( PythonLibs 2.7 REQUIRED )
IF(PYTHONLIBS_FOUND)
  SET(HAVE_PYTHON 1)
ELSE(PYTHONLIBS_FOUND)
  SET(HAVE_PYTHON 0)
ENDIF(PYTHONLIBS_FOUND)
INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_DIRS})

#motion library: client
ADD_LIBRARY(motion_client SHARED
		Motion/motion_client.cpp)
TARGET_LINK_LIBRARIES(motion_client ${KRISLIBRARY_LIBRARIES})

#motion library: kinematic
ADD_LIBRARY(motion_kinematic SHARED
		Motion/motion_kinematic.cpp)
TARGET_LINK_LIBRARIES(motion_kinematic ${SSPP_LIBRARIES} ${KLAMPT_LIBRARIES})

#motion library: physics simulation (not done yet)
#ADD_LIBRARY(motion_klamptsim SHARED
#		Motion/motion_klamptsim.cpp)
#TARGET_LINK_LIBRARIES(motion_klamptsim ${SSPP_LIBRARIES}) ${KLAMPT_LIBRARIES})


#motion server: virtual robot
ADD_EXECUTABLE(MotionServer_kinematic 
		Motion/motion_server_kinematic.cpp Motion/motion_server.cpp)
TARGET_LINK_LIBRARIES(MotionServer_kinematic ${SSPP_LIBRARIES} ${KLAMPT_LIBRARIES})

#motion server: physics simulation (not done yet)
#ADD_EXECUTABLE(MotionServer_klamptsim 
#		Motion/motion_server_klamptsim.cpp Motion/motion_server.cpp)
#TARGET_LINK_LIBRARIES(MotionServer_klamptsim ${SSPP_LIBRARIES}) ${KLAMPT_LIBRARIES})

#motion library: real robot
SET (CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/CMakeModules")
SET(ROSDEPS rosconsole roscpp roscpp_serialization rostime )
FIND_PACKAGE(ROS)
IF(ROS_FOUND)
  MESSAGE("ROS FOUND! path ${ROS_PATH}")
  SET(ROS_WS /home/motion/ros_ws CACHE STRING "ROS workspace including Baxter SDK")
  INCLUDE_DIRECTORIES(${ROS_PATH}/include ${ROS_WS}/install/include)

  #motion server: physical robot
  ADD_EXECUTABLE(MotionServer_physical 
    Motion/motion_server_physical.cpp Motion/motion_server.cpp)
  TARGET_LINK_LIBRARIES(MotionServer_physical ${ROS_LIBRARIES} ${SSPP_LIBRARIES} ${KLAMPT_LIBRARIES})

  #motion library: physical robot 
  ADD_LIBRARY(motion_physical SHARED
    Motion/motion_physical.cpp)
  TARGET_LINK_LIBRARIES(motion_physical ${ROS_LIBRARIES} ${SSPP_LIBRARIES} ${KLAMPT_LIBRARIES})

  ADD_EXECUTABLE(RealSense_ROS_Emitter Sensors/realsense_ros_emitter.cpp)
  TARGET_LINK_LIBRARIES(RealSense_ROS_Emitter ${KRISLIBRARY_LIBRARIES} ${ROS_LIBRARIES})
ENDIF(ROS_FOUND)
