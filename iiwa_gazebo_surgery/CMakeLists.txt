cmake_minimum_required(VERSION 3.5)
project(iiwa_gazebo_surgery)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  iiwa_ros
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
)

catkin_package()

catkin_install_python(PROGRAMS scripts/surgery_moveit.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_install_python(PROGRAMS scripts/spawn_tool.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
