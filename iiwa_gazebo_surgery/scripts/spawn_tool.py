#!/usr/bin/env python

import rospy
import rospkg
import time
import geometry_msgs.msg
from rospy.core import rospyinfo
from tf.transformations import quaternion_from_euler
from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.srv import DeleteModel
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.srv import GetWorldProperties
from std_srvs.srv import Empty
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse

#-y -0.0 -x 0.2 -z 2.3 -R 0.0 -P 0.0 -Y 0.0


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
    tool_pose.position.x = 0.2
    tool_pose.position.y = 0
    tool_pose.position.z = 2.23 + 0.05
    if tool_name == 'pen_support':
        tool_pose.position.z -= 0.01
    if tool_name == 'tms':
        tool_pose.position.z -= 0.02
    tool_pose_r = 0
    tool_pose_p = 0
    tool_pose_y = 0
    quaternion = quaternion_from_euler(tool_pose_r, tool_pose_p, tool_pose_y)
    tool_pose.orientation.x = quaternion[0]
    tool_pose.orientation.y = quaternion[1]
    tool_pose.orientation.z = quaternion[2]
    tool_pose.orientation.w = quaternion[3]

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
    # tool_name = 'pen_support'
    tool_name = 'tms'
    check_result = check_iiwa_tool(tool_name)
    if check_result:
        rospy.loginfo('Spawning %s' % tool_name)
        spawn_tool(tool_name)
        unpause_physics_client=rospy.ServiceProxy('/gazebo/unpause_physics',Empty)
        unpause_physics_client.wait_for_service()
        unpause = unpause_physics_client()