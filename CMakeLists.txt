cmake_minimum_required(VERSION 3.0.2)
project(motion_capture)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  gazebo_ros
  sensor_msgs
  octomap_ros
  octomap_msgs
)

catkin_package(
  CATKIN_DEPENDS
  gazebo_ros
)

find_package(octomap REQUIRED)

include_directories(${OCTOMAP_INCLUDE_DIRS})


install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY sdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)