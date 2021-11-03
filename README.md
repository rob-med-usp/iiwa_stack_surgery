# iiwa_stack_surgery
Packages on how to extend [**iiwa_stack**](https://github.com/SalvoVirga/iiwa_stack) for the use in neurosurgical application
    
[**iiwa_stack**](https://github.com/SalvoVirga/iiwa_stack) needs to be present on the system to make these packages work.

In brief, here is contained :
- **iiwa_gazebo_surgery**: some modified files from iiwa_stack and scripts to communicate both with the simulation and moveit
- **iiwa_surgery_description** : description of the robot on the support without any tools
- **iiwa_surgery_moveit** : moveit package to control the robot using moveit
- **operating_room** : definition of the static objects and their properties within the scene where the robot will be
- **gazebo_ros_link_attacher**: world plugin and example python codes for attaching and detaching links from different models. Used to attach and detach tools to the robot.


### Moveit - GUI
```
roslaunch iiwa_gazebo_surgery iiwa_gazebo_with_sunrise.launch
roslaunch iiwa_gripper_moveit moveit_planning_execution.launch
```

### Moveit - ROS node
```
roslaunch iiwa_gazebo_surgery iiwa_gazebo_with_sunrise.launch
roslaunch iiwa_gazebo_surgery basis_change_moveit.launch
```
