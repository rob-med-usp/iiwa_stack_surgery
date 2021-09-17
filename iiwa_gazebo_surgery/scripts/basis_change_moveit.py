#!/usr/bin/env python

import time
from math import pi
import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler


def go_to_pose(head_basis):
    
    world_basis = head_basis    
    world_basis.position.x += 0.0
    world_basis.position.y += 0.0
    world_basis.position.z += 0.0

    # Pose Orientation
    roll_angle = 0.8
    pitch_angle = 0.8
    yaw_angle = 0.8
    quaternion = quaternion_from_euler(roll_angle, pitch_angle, yaw_angle)

    world_basis.orientation.x = quaternion[0]
    world_basis.orientation.y = quaternion[1]
    world_basis.orientation.z = quaternion[2]
    world_basis.orientation.w = quaternion[3]

    print "Pose and orientetion sent \n", world_basis


    group.set_pose_target(world_basis)

    ## Now, we call the planner to compute the plan and execute it
    plan = group.plan()


    print "====== type 1 to execute the planned trajectory"
    print "====== type any other key to cancel the plan"
    answer = raw_input('Input: ')
    if answer != "1":
        print "====== canceling trajectory, please publish another pose goal"
        return
    
    group.execute(plan, wait=True)
    # Calling `stop()` ensures that there is no residual movement
    #group.stop()
    # It is always good to clear your targets after planning with poses
    #group.clear_pose_targets()

if __name__ == "__main__":
    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('change_of_basis_moveit', anonymous=True)
    
    display_trajectory_publisher = rospy.Publisher('/iiwa/move_group/display_planned_path',
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)
    ## Subscriber that receives head coordinates and call a function to compute
    ## the basis change and execute it of ok
    pose_sub = rospy.Subscriber('/HeadCoordinates', geometry_msgs.msg.Pose, go_to_pose)
    
    ## Instantiate a `RobotCommander`_ object. This object is the outer-level interface to
    ## the robot:
    robot = moveit_commander.RobotCommander()

    ## Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
    ## to the world surrounding the robot:
    scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to one group of joints. 
    group_name = "manipulator"
    group = moveit_commander.MoveGroupCommander(group_name)

    rospy.spin()