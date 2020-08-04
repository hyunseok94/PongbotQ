cmake_minimum_required(VERSION 2.8.3)
project(pongbot_q_v1)

# for RBDL
LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR} )

FIND_PACKAGE (RBDL REQUIRED)
FIND_PACKAGE (Eigen3 3.0.0 REQUIRED)
# Find OSQP library and headers
FIND_PACKAGE (osqp REQUIRED)


find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
)


find_package(gazebo REQUIRED)
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${PCL_LIBRARY_DIRS} ${RBDL_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR} ${ADDITIONAL_INCLUDE_DIRS} ${OGRE_INCLUDE_DIRS}/Paging)

add_library(${PROJECT_NAME} src/pongbotq_plugin.cc src/CRobot.cpp src/QuadProgpp/Array.cc src/QuadProgpp/QuadProg++.cc)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${RBDL_LIBRARY} ${RBDL_URDFReader_LIBRARY})
target_link_libraries(${PROJECT_NAME} PRIVATE osqp::osqp)

catkin_package(
#   INCLUDE_DIRS include
   LIBRARIES pongbot_q_v1
   DEPENDS gazebo_ros roscpp rospy std_msgs sensor_msgs
#  DEPENDS system_lib
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")


