#!/usr/bin/env python

import time
from math import pi
import rospy

from iiwa_msgs.msg import CartesianPose
from geometry_msgs.msg import PoseStamped


def pose_publisher(head_basis):
    robot_basis = head_basis
    robot_basis.pose.position.x += 0.0
    robot_basis.pose.position.y += 0.0
    pose_pub.publish(robot_basis)

if __name__ == "__main__":

    rospy.init_node('change_of_basis', anonymous=True)
    pose_pub = rospy.Publisher('/iiwa/command/CartesianPose', PoseStamped, queue_size = 10)
    pose_sub = rospy.Subscriber('/HeadCoordinates', PoseStamped, pose_publisher)
    rospy.spin()
