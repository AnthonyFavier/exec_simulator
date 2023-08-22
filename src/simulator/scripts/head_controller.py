#!/usr/bin/env python3
import rospy
import typing
from std_msgs.msg import String 
from geometry_msgs.msg import Pose, Vector3, Quaternion
from sim_msgs.msg import HeadCmd
import math
from scipy.spatial.transform import Rotation as R
from gazebo_msgs.srv import SetModelState, SetModelStateRequest, SetModelStateResponse
from gazebo_msgs.srv import SetLinkState, SetLinkStateRequest, SetLinkStateResponse
from gazebo_msgs.srv import GetModelState, GetModelStateRequest, GetModelStateResponse

###########################

def compute_rpy_to_target_pose(target_pose: Pose):
    v = Vector3()
    v.x = target_pose.position.x - g_head_pose.position.x 
    v.y = target_pose.position.y - g_head_pose.position.y 
    v.z = target_pose.position.z - g_head_pose.position.z 


    # suivant 'x'
    roll = 0

    # suivant 'y'
    if v.x==0 and v.z==0:
        pitch=0
    else:
        pitch = -math.acos(v.x/math.sqrt(pow(v.x,2) + pow(v.z,2)))
        if v.z<0:
            pitch = -pitch

    # suivant 'z'
    if v.x==0 and v.y==0:
        yaw=0
    else:
        yaw = math.acos(v.x/math.sqrt(pow(v.x,2) + pow(v.y,2)))
        if v.y<0:
            yaw = -yaw

    return (roll, pitch, yaw)

def rotate_to_target(target_pose: Pose):
    r,p,y = compute_rpy_to_target_pose(target_pose)
    rot = R.from_euler('xyz', [r,p,y])
    q = rot.as_quat()

    req = GetModelStateRequest()
    req.model_name = "robot_head"
    resp = g_get_model_state_prox.call(req) # type: GetModelStateResponse
    curr_q = resp.pose.orientation

    dir_q = Quaternion()
    dir_q.x = q[0] - curr_q.x
    dir_q.y = q[1] - curr_q.y
    dir_q.z = q[2] - curr_q.z
    dir_q.w = q[3] - curr_q.w

    norm = math.sqrt(pow(dir_q.x,2) + pow(dir_q.y,2) + pow(dir_q.z,2) + pow(dir_q.w,2))

    tolerance = 0.01
    goal_reached = abs(dir_q.x) <= tolerance and abs(dir_q.y) <= tolerance and abs(dir_q.z) <= tolerance and abs(dir_q.w) <= tolerance

    n_dir_q = Quaternion()
    n_dir_q.x = dir_q.x / norm
    n_dir_q.y = dir_q.y / norm
    n_dir_q.z = dir_q.z / norm
    n_dir_q.w = dir_q.w / norm

    loop = rospy.Rate(50)
    step = 0.01
    tolerance = 0.01
    while not rospy.is_shutdown() and not goal_reached:
        
        curr_q.x += n_dir_q.x * step
        curr_q.y += n_dir_q.y * step
        curr_q.z += n_dir_q.z * step
        curr_q.w += n_dir_q.w * step

        req_head = SetModelStateRequest()
        req_head.model_state.model_name = "robot_head"
        req_head.model_state.pose.position = g_head_pose.position
        req_head.model_state.pose.orientation.x = curr_q.x
        req_head.model_state.pose.orientation.y = curr_q.y
        req_head.model_state.pose.orientation.z = curr_q.z
        req_head.model_state.pose.orientation.w = curr_q.w
        g_set_model_state_prox.call(req_head)

        goal_reached =  abs(q[0] - curr_q.x)<=tolerance\
                    and abs(q[1] - curr_q.y)<=tolerance\
                    and abs(q[2] - curr_q.z)<=tolerance\
                    and abs(q[3] - curr_q.w)<=tolerance\

        loop.sleep()

###########################

def follow_obj(obj_name):
    global new_cmd

    loop = rospy.Rate(30)

    new_cmd = False

    req_obj = GetModelStateRequest()
    req_obj.model_name = obj_name

    req_head = SetModelStateRequest()
    req_head.model_state.model_name = "robot_head"
    req_head.model_state.pose.position = g_head_pose.position

    # req_head = SetLinkStateRequest()
    # req_head.link_state.link_name = "robot_head_link"
    # req_head.link_state.pose.position = g_head_pose.position


    while not rospy.is_shutdown() and not new_cmd:
        
        resp = g_get_model_state_prox.call(req_obj) # type: GetModelStateResponse
        print(f"obj_pose = {resp.pose.position}")

        r,p,y = compute_rpy_to_target_pose(resp.pose)
        
        rot = R.from_euler('xyz', [r,p,y])
        q = rot.as_quat()

        req_head.model_state.pose.orientation.x = q[0]
        req_head.model_state.pose.orientation.y = q[1]
        req_head.model_state.pose.orientation.z = q[2]
        req_head.model_state.pose.orientation.w = q[3]
        g_set_model_state_prox.call(req_head)

        # req_head.link_state.pose.orientation.x = q[0]
        # req_head.link_state.pose.orientation.y = q[1]
        # req_head.link_state.pose.orientation.z = q[2]
        # req_head.link_state.pose.orientation.w = q[3]
        # g_set_link_state_prox.call(req_head)

        print(f"updated {p} {y}")
        loop.sleep()

def look_at_obj(obj_name):
    global new_cmd

    new_cmd = False

    rospy.loginfo("START LOOK_AT_OBJ")

    req_obj = GetModelStateRequest()
    req_obj.model_name = obj_name
    resp = g_get_model_state_prox.call(req_obj) # type: GetModelStateResponse
    print(f"obj_pose = {resp.pose.position}")
        
    rospy.loginfo("START ROTATE")
    rotate_to_target(resp.pose)
    rospy.loginfo("END ROTATE")

    rospy.loginfo("END LOOK_AT_OBJ")


def look_at_stack():
    global new_cmd

    new_cmd = False

    rotate_to_target(g_stack_pose)

def look_at_pose(pose):
    global new_cmd

    new_cmd = False

    rotate_to_target(pose)


def be_passive():
    pass

def wait_h():
    global new_cmd

    new_cmd = False

    rotate_to_target(g_camera_pose)

def reset():
    global new_cmd

    new_cmd = False

    p = g_head_pose
    p.position.x += 1.0

    rotate_to_target(p)

def cmd_cb(cmd: HeadCmd):
    global new_cmd

    new_cmd = True
    rospy.sleep(0.1)
    print("Head Cmd received !")

    if HeadCmd.FOLLOW_OBJ == cmd.type:
        follow_obj(cmd.obj_name)
    if HeadCmd.LOOK_AT_OBJ == cmd.type:
        look_at_obj(cmd.obj_name)
    if HeadCmd.LOOK_AT_STACK == cmd.type:
        look_at_stack()
    elif HeadCmd.BE_PASSIVE == cmd.type:
        be_passive()
    elif HeadCmd.WAIT_H == cmd.type:
        wait_h()
    elif HeadCmd.LOOK_AT_POSE == cmd.type:
        look_at_pose(cmd.pose)
    elif HeadCmd.RESET == cmd.type:
        reset()

if __name__ == '__main__':
    rospy.init_node('head_controller')

    cmd_sub = rospy.Subscriber("/head_cmd", HeadCmd, cmd_cb, queue_size=10)
    rospy.wait_for_service("/gazebo/set_model_state")
    g_set_model_state_prox = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
    rospy.wait_for_service("/gazebo/set_link_state")
    g_set_link_state_prox = rospy.ServiceProxy("/gazebo/set_link_state", SetLinkState)
    rospy.wait_for_service("/gazebo/get_model_state")
    g_get_model_state_prox = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)

    req = GetModelStateRequest()
    req.model_name = "robot_head"
    resp = g_get_model_state_prox.call(req) # type: GetModelStateResponse
    g_head_pose = resp.pose
    print(f"g_head_pose={g_head_pose}")

    g_camera_pose = Pose()
    g_camera_pose.position.x = 2.237550
    g_camera_pose.position.y = 0
    g_camera_pose.position.z = 2.492110

    g_stack_pose = Pose()
    g_stack_pose.position.x = 0.86
    g_stack_pose.position.y = 0.24
    g_stack_pose.position.z = 0.7

    new_cmd = False

    print("Head controller ready")

    loop = rospy.Rate(50)

    rospy.spin()
