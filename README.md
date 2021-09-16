# iiwa_stack_examples
Packages on how to extend [**iiwa_stack**](https://github.com/SalvoVirga/iiwa_stack) for the use in neurosurgical application

This packages are used within the [**wiki of iiwa_stack**](https://github.com/SalvoVirga/iiwa_stack/wiki) to explain some concepts of its usage.     
[**iiwa_stack**](https://github.com/SalvoVirga/iiwa_stack) needs to be present on the system to make these packages work.

In brief, here is contained :
- **iiwa_gazebo_surgery**: modified files from iiwa_stack and some more files to run the simulations
- **iiwa_gripper_description** : description of the robot with a gripper attached to it - the configuration is done as the gripper did not move, the initial goal its just to take its dimensions into consideration when planning trajectory
- **iiwa_gripper_moveit** : moveit packege to control the robot with the gripper using moveit
- **operating_room** : definition of the static objects and their properties within the scene where the robot will be


### Moveit
```
roslaunch iiwa_gazebo_surgery iiwa_gazebo_with_sunrise.launch
roslaunch iiwa_gripper_moveit moveit_planning_execution.launch
```
