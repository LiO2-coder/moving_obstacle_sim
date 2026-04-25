cmake_minimum_required(VERSION 3.0.2)
project(moving_obstacle_sim)

if(CMAKE_VERSION VERSION_LESS "3.1")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
else()
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  gazebo_msgs
  roscpp
  rospy
)

find_package(gazebo REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES moving_obstacle_plugin
  CATKIN_DEPENDS gazebo_ros gazebo_msgs roscpp rospy
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(moving_obstacle_plugin src/moving_obstacle_plugin.cpp)
target_link_libraries(moving_obstacle_plugin
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)

catkin_install_python(PROGRAMS
  scripts/spawn_moving_obstacles.py
  scripts/model_builders.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS moving_obstacle_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config models urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
