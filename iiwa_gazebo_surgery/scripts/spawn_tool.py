#!/usr/bin/env python

import rospy
import rospkg
import time
import math
import geometry_msgs.msg
from rospy.core import rospyinfo
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.srv import DeleteModel
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.srv import GetLinkState
from gazebo_msgs.srv import GetWorldProperties
from std_srvs.srv import Empty
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse

#-y -0.0 -x 0.2 -z 2.3 -R 0.0 -P 0.0 -Y 0.0
def get_iiwa_link_state():
    get_state = rospy.ServiceProxy('gazebo/get_link_state', GetLinkState)
    iiwa_link_state = get_state('iiwa_link_7','link_suporte')
    print iiwa_link_state.link_state.pose
    return iiwa_link_state.link_state.pose

    # It is possible getting the end effector position using moveit, but it has to bu ran under the namespace 'iiwa'
    # group_name = "iiwa_surgery"
    # group = moveit_commander.MoveGroupCommander(group_name)
    # iiwa_pose = group.get_current_pose('iiwa_link_ee')
    # return iiwa_pose.pose

def q_mult(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
    z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2
    return w, x, y, z

def q_conjugate(q):
    w, x, y, z = q
    return (w, -x, -y, -z)

def qv_mult(q1, v1):
    q2 = (0.0,) + v1
    return q_mult(q_mult(q1, q2), q_conjugate(q1))[1:]

def ee_pose(z):
    iiwa_ee_pose = get_iiwa_link_state()
    # orientation = euler_from_quaternion([iiwa_ee_pose.orientation.x, iiwa_ee_pose.orientation.y, iiwa_ee_pose.orientation.z, iiwa_ee_pose.orientation.w])
    # iiwa_ee_pose.position.x += z*math.cos(orientation[1])*math.sin(orientation[2])
    # iiwa_ee_pose.position.y += z*math.cos(orientation[2])*math.cos(orientation[1])
    # iiwa_ee_pose.position.z += z*math.sin(orientation[1])
    v = (0,0,z)
    iiwa_q = (iiwa_ee_pose.orientation.w, iiwa_ee_pose.orientation.x, iiwa_ee_pose.orientation.y, iiwa_ee_pose.orientation.z)
    rotation = qv_mult(iiwa_q, v)
    print rotation
    iiwa_ee_pose.position.x += rotation[0]
    iiwa_ee_pose.position.y += rotation[1]
    iiwa_ee_pose.position.z += rotation[2]

    print iiwa_ee_pose
    return iiwa_ee_pose

def change_tool():
    # tool_name = 'pen_support'
    tool_name = 'drill'
    check_result = check_iiwa_tool(tool_name)
    if check_result:
        rospy.loginfo('Spawning %s' % tool_name)
        spawn_tool(tool_name)
        unpause_physics_client=rospy.ServiceProxy('/gazebo/unpause_physics',Empty)
        unpause_physics_client.wait_for_service()
        unpause = unpause_physics_client()

def check_iiwa_tool(toolname):
    world_properties_prox = rospy.ServiceProxy('gazebo/get_world_properties', GetWorldProperties)
    world_properties_prox.wait_for_service()
    world_properties = world_properties_prox()

    tool_present = False
    if 'drill' in world_properties.model_names:
        if 'drill' == toolname:
            rospy.loginfo('drill already in simulation, nothing added')
            return False
        req = AttachRequest()
        req.model_name_2 = "drill"
        req.link_name_2 = "drill_link"
        tool_present = True
    elif 'pen_support' in world_properties.model_names:
        if 'pen_support' == toolname:
            rospy.loginfo('pen_support already in simulation, nothing added')
            return False
        req = AttachRequest()
        req.model_name_2 = "pen_support"
        req.link_name_2 = "pen_support_link"
        tool_present = True
    elif 'tms' in world_properties.model_names:
        if 'tms' == toolname:
                rospy.loginfo('tms already in simulation, nothing added')
                return False
        req = AttachRequest()
        req.model_name_2 = "tms"
        req.link_name_2 = "tms_link"
        tool_present = True
    else:
        rospy.loginfo('No tool already present in simulation, proceeding to spawning and attaching requested tool')
        pause_physics_client=rospy.ServiceProxy('/gazebo/pause_physics',Empty)
        pause_physics_client.wait_for_service()
        pause = pause_physics_client()
        return True

    if tool_present:
        pause_physics_client = rospy.ServiceProxy('/gazebo/pause_physics',Empty)
        pause = pause_physics_client()
        
        rospy.loginfo('Detaching %s' % req.model_name_2)
        req.model_name_1 = "iiwa"
        req.link_name_1 = "iiwa_link_7"
        detach_srv = rospy.ServiceProxy('/link_attacher_node/detach', Attach)
        detach_srv.wait_for_service()
        detach_srv.call(req)
        rospy.loginfo('Deleting %s' % req.model_name_2)
        delete_model(req.model_name_2)
        return True

def spawn_tool(toolname):
    tool_pose = geometry_msgs.msg.Pose()
    tool_pose = ee_pose(0.05)
    # tool_pose.position.z += 0.05
    # if toolname == 'pen_support':
    #     tool_pose.position.z -= 0.01
    # if toolname == 'tms':
    #     tool_pose.position.z -= 0.02
    # tool_pose_r = 0
    # tool_pose_p = 0
    # tool_pose_y = 0
    # quaternion = quaternion_from_euler(tool_pose_r, tool_pose_p, tool_pose_y)
    # tool_pose.orientation.x = quaternion[0]
    # tool_pose.orientation.y = quaternion[1]
    # tool_pose.orientation.z = quaternion[2]
    # tool_pose.orientation.w = quaternion[3]

    tool_path = rospkg.RosPack().get_path('operating_room') + '/models/' + toolname + '/model.sdf'

    with open (tool_path, 'r') as xml_file:
        model_xml = xml_file.read().replace('\n', '')

    spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
    spawn_model_prox(toolname, model_xml, '', tool_pose, 'link_suporte')

    attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
    attach_srv.wait_for_service()
    rospy.loginfo('Attaching iiwa link 7 and %s' % toolname)
    req = AttachRequest()
    req.model_name_1 = "iiwa"
    req.link_name_1 = "iiwa_link_7"
    req.model_name_2 = toolname
    req.link_name_2 = toolname + '_link'
    attach_srv.call(req)

def delete_model(toolname):
    # Delete the old model if it's stil around
    delete_model_prox = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
    delete_model_prox(toolname)


if __name__ == "__main__":
    rospy.init_node('tool_spawner', anonymous=True)
    change_tool()
    # get_iiwa_link_state()
    # ee_pose(0.05)