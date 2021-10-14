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
    
    world_basis = geometry_msgs.msg.Pose()    
    world_basis.position.x = head_basis.linear.x
    world_basis.position.y = head_basis.linear.y
    world_basis.position.z = head_basis.linear.z

    # Pose Orientation
    roll_angle = head_basis.angular.x
    pitch_angle = head_basis.angular.y
    yaw_angle = head_basis.angular.z
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
    group.stop()
    # It is always good to clear your targets after planning with poses
    group.clear_pose_targets()

def add_objects():
    ## Add "head" (a sphere so far)
    print "started adding head"
    head_pose = geometry_msgs.msg.PoseStamped()
    head_pose.header.frame_id = "world"
    head_pose.pose.position.x = 0.2
    head_pose.pose.position.y = 0.8
    head_pose.pose.position.z = 0.83
    head_pose.pose.orientation.w = 1.0
    scene.add_sphere("head", head_pose, radius = 0.09)
    # wait 2 seconds for it to be added
    head_included = object_in_scene(name="head", timeout=2)

    ## Add the operating table
    print "started adding operating table"
    operating_table_pose = geometry_msgs.msg.PoseStamped()
    operating_table_pose.header.frame_id = "world"
    operating_table_pose.pose.position.x = -0.1
    operating_table_pose.pose.position.y = 0.4
    operating_table_pose.pose.position.z = 0
    quaternion = quaternion_from_euler(1.57, 0, 1.57)
    operating_table_pose.pose.orientation.x = quaternion[0]
    operating_table_pose.pose.orientation.y = quaternion[1]
    operating_table_pose.pose.orientation.z = quaternion[2]
    operating_table_pose.pose.orientation.w = quaternion[3]
    operating_table_path = "/home/lincoln/iiwa_stack_ws/src/iiwa_stack_surgery/operating_room/models/operating_table/meshes/medical_operating_table.stl"
    scene.add_mesh("operating_table", operating_table_pose, operating_table_path, size = (0.001, 0.001, 0.001))
    # wait 2 seconds for it to be added
    operating_table_included = object_in_scene(name="operating_table", timeout=2)

def object_in_scene(name="", timeout=2):
    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
        # Test if the box is in the scene.
        # Note that attaching the box will remove it from known_objects
        is_known = name in scene.get_known_object_names()

        # Test if we are in the expected state
        if is_known:
            print name, " included in the scene"
            return True

        # Sleep so that we give other threads time on the processor
        rospy.sleep(0.1)
        seconds = rospy.get_time()

    # If we exited the while loop without returning then we timed out
    print "head not included in the scene"
    return False


if __name__ == "__main__":
    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('change_of_basis_moveit', anonymous=True)
    
    display_trajectory_publisher = rospy.Publisher('/iiwa/move_group/display_planned_path',
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)
    ## Subscriber that receives head coordinates and call a function to compute
    ## the basis change and execute it of ok
    pose_sub = rospy.Subscriber('/HeadCoordinates', geometry_msgs.msg.Twist, go_to_pose)
    
    ## Instantiate a `RobotCommander`_ object. This object is the outer-level interface to
    ## the robot:
    robot = moveit_commander.RobotCommander()

    ## Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
    ## to the world surrounding the robot:
    scene = moveit_commander.PlanningSceneInterface(synchronous=True)

    # Add static objects to the scene
    # add_objects()
    # scene.remove_world_object("head")
    # scene.remove_world_object("operating_table")

    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to one group of joints. 
    group_name = "manipulator"
    group = moveit_commander.MoveGroupCommander(group_name)

    group.set_planning_time(30)
    # group.set_planner_id("KPIECE")
    group.set_planner_id("RRTstarkConfigDefault")

    rospy.spin()