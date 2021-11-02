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

catkin_install_python(PROGRAMS scripts/planning_execution.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_install_python(PROGRAMS scripts/basis_change_moveit.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
