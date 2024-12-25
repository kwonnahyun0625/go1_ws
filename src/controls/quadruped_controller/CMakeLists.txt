cmake_minimum_required(VERSION 3.0.2)
project(go1_controller)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  std_msgs
  roscpp
  roslib
  rospy
  sensor_msgs
  urdf
  tf
  xacro
  tf_conversions
  gazebo_ros
  controller_manager
)

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_PACKAGE_DEPENDENCIES}
)
  
find_package(PkgConfig REQUIRED)
pkg_check_modules(LCM REQUIRED lcm)

find_package(gazebo REQUIRED)


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS eigen_conversions geometry_msgs sensor_msgs tf tf_conversions
 DEPENDS gazebo_ros roscpp rospy std_msgs Boost  
)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${LCM_INCLUDE_DIRS})
link_directories(${LCM_LIBRARY_DIRS})

include_directories(
  ${Boost_INCLUDE_DIR} 
  ${catkin_INCLUDE_DIRS} 
  ${GAZEBO_INCLUDE_DIRS} 
  ${PCL_LIBRARY_DIRS} 
  ${EIGEN3_INCLUDE_DIR} 
  )
 

include_directories(
  include
  include/Tools
  ${catkin_INCLUDE_DIRS}
)

add_subdirectory(include/Tools)

add_library(${PROJECT_NAME}
  include/go1_controller.h
  src/go1_controller.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${LCM_INCLUDE_DIRS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_executable(go1_node src/go1_controller_node.cpp)
add_dependencies(go1_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(go1_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

target_link_libraries(${PROJECT_NAME} Tools ${LCM_LIBRARIES})

install(DIRECTORY launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)