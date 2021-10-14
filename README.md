# iiwa_stack_surgery
Packages on how to extend [**iiwa_stack**](https://github.com/SalvoVirga/iiwa_stack) for the use in neurosurgical application
    
[**iiwa_stack**](https://github.com/SalvoVirga/iiwa_stack) needs to be present on the system to make these packages work.

In brief, here is contained :
- **iiwa_gazebo_surgery**: modified files from iiwa_stack and some more files to run the simulations
- **iiwa_gripper_description** : description of the robot with a gripper attached to it - the configuration is done as the gripper did not move, the initial goal its just to take its dimensions into consideration when planning trajectory
- **iiwa_gripper_moveit** : moveit packege to control the robot with the gripper using moveit
- **operating_room** : definition of the static objects and their properties within the scene where the robot will be
- **gazebo_ros_link_attacher**: world plugin and example python codes for attaching and detaching links from different models. Used to attach and detach tools to the robot.


### Moveit
```
roslaunch iiwa_gazebo_surgery iiwa_gazebo_with_sunrise.launch
roslaunch iiwa_gripper_moveit moveit_planning_execution.launch
```
