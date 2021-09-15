cmake_minimum_required(VERSION 3.1.3)
project(iiwa_gripper_moveit)

find_package(catkin REQUIRED)

catkin_package()

add_executable(command_robot src/command_robot.cpp)
add_dependencies(command_robot iiwa_msgs_generate_messages_cpp)
target_link_libraries(command_robot ${catkin_LIBRARIES})

add_executable(command_robot_moveit src/command_robot_moveit.cpp)
add_dependencies(command_robot_moveit iiwa_msgs_generate_messages_cpp)
target_link_libraries(command_robot_moveit ${catkin_LIBRARIES})

add_executable(setting_control_mode src/setting_control_mode.cpp)
add_dependencies(setting_control_mode iiwa_msgs_generate_messages_cpp)
target_link_libraries(setting_control_mode ${catkin_LIBRARIES})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
