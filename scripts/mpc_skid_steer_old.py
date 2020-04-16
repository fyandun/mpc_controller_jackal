#!/usr/bin/env python
import rospy
import numpy as np
import acado
import math

import robot_common.global_defs as defs
import robot_common.utils as utils
import robot_common.robot_motion_skid_steer as bot_model

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Pose, Twist
import tf


first_seen = False
isInitState = True

#initial robot state
robot_state = bot_model.kinematics(0, 0, 0, 0)
last_time = rospy.Time()
dt = 0.1


def iterative_linear_mpc_control(xref, dref, oa, ow):
    """
    MPC contorl with updating operational point iteraitvely
    """
    x0 = robot_state.get_current_meas_state()
    if oa is None or ow is None:
        oa = [0.0] * defs.T
        ow = [0.0] * defs.T

    for i in range(defs.MAX_ITER):
        xbar = robot_state.predict_motion(oa, ow, defs.T)
        #print(xbar)
        poa, podw = oa[:], ow[:]
        oa, ow, ox, oy, oyaw, ov = linear_mpc_control(xref, xbar, x0, dref)
        du = sum(abs(oa - poa)) + sum(abs(ow - podw))  # calc u change value
        if du <= defs.DU_TH:
            break
    else:
        print("Iterative is max iter")

    robot_state.refreshState()
    return oa, ow, ox, oy, oyaw, ov

# MPC using ACADO
def linear_mpc_control(xref, xbar, x0, dref):
    # see acado.c for parameter details
    _x0=np.zeros((1, defs.NX))  
    X=np.zeros((defs.T+1, defs.NX))
    U=np.zeros((defs.T, defs.NU))    
    Y=np.zeros((defs.T, defs.NY))    
    yN=np.zeros((1, defs.NYN))    
    _x0[0,:]=np.transpose(x0)  # initial state    
    for t in range(defs.T):
      Y[t,:] = np.transpose(xref[:,t])  # reference state
      X[t,:] = np.transpose(xbar[:,t])  # predicted state
    X[-1,:] = X[-2,:]    
    yN[0,:]=Y[-1,:defs.NYN]         # reference terminal state
    X, U = acado.mpc(0, 1, _x0, X,U,Y,yN, np.transpose(np.tile(defs.Q,defs.T)), defs.Qf, 0)    
    ox_mpc = utils.get_nparray_from_matrix(X[:,0])
    oy_mpc = utils.get_nparray_from_matrix(X[:,1])
    ov_mpc = utils.get_nparray_from_matrix(X[:,2])
    oyaw_mpc = utils.get_nparray_from_matrix(X[:,3])
    oa_mpc = utils.get_nparray_from_matrix(U[:,0])
    ow_mpc = utils.get_nparray_from_matrix(U[:,1])
    return oa_mpc, ow_mpc, ox_mpc, oy_mpc, oyaw_mpc, ov_mpc    
    
def callbackFilteredOdom(odom_msg):
    global last_time
    global dt
    current_time = rospy.Time.now()
    #robotPoseEstimate = PoseStamped()
    #robotPoseEstimate.pose.position = odom_msg.pose.pose.position
    # robotPoseEstimate.pose.orientation = odom_msg.pose.pose.orientation
    x_meas = odom_msg.pose.pose.position.x
    y_meas = odom_msg.pose.pose.position.y
    quat_pose = (
        odom_msg.pose.pose.orientation.x,
        odom_msg.pose.pose.orientation.y,
        odom_msg.pose.pose.orientation.z,
        odom_msg.pose.pose.orientation.w)
    euler_meas = tf.transformations.euler_from_quaternion(quat_pose) #RPY

    v_meas = odom_msg.twist.twist.linear.x
    w_meas = odom_msg.twist.twist.angular.z

    if (last_time.to_nsec()==0):
        print("Here")
        robot_state.update_state(x_meas, y_meas, euler_meas[2], v_meas, dt)
        last_time = current_time
    elif (last_time.to_nsec() > 0):
        dt = current_time.to_sec() - last_time.to_sec() 
        if (~robot_state.IsFresh and dt>=0.1):
            print(dt)
            robot_state.update_state(x_meas, y_meas, euler_meas[2], v_meas, dt)
            last_time = current_time
    
    
    

#here I need to create the msg to send - chech in case of a carlike
def make_twist_msg(accel, omega):
    cmd = Twist()
    cmd.linear.x = accel*dt
    cmd.linear.y = 0
    cmd.linear.z = 0
    cmd.angular.x = 0
    cmd.angular.y = 0
    cmd.angular.z = omega
    return cmd


def mpc_node():
    rospy.init_node('mpc_jackal', anonymous=True)

    odomSubs = rospy.Subscriber("/odometry/filtered", Odometry, callbackFilteredOdom)
    controlPub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    pathPub = rospy.Publisher("/aPath", Path, queue_size=10)
    
    #generate the paths
    dl =0.1
    #cx, cy, cyaw, ck = utils.get_straight_course(dl)
    #cx, cy, cyaw, ck = utils.get_straight_course2(dl)
    #cx, cy, cyaw, ck = utils.get_straight_course3(dl)
    cx, cy, cyaw, ck = utils.get_forward_course(dl)
    #cx, cy, cyaw, ck = utils.get_switch_back_course(dl)

    sp = utils.calc_speed_profile(cx, cy, cyaw, defs.TARGET_SPEED)
    rate = rospy.Rate(10) # 10hz
    goal = [cx[-1], cy[-1]]
        
    my_path = Path()
    my_path.header.frame_id = 'odom'
    for x,y in zip(cx, cy):
        pose = PoseStamped()
        pose.pose.position.x = x
        pose.pose.position.y = y        
        my_path.poses.append(pose)
    

    # initial yaw compensation
    if robot_state.yaw - cyaw[0] >= math.pi:
        robot_state.yaw -= math.pi * 2.0
    elif robot_state.yaw - cyaw[0] <= -math.pi:
        robot_state.yaw += math.pi * 2.0

    target_ind, _ = utils.calc_nearest_index(robot_state, cx, cy, cyaw, 0)
    
    ow, oa = None, None
    cyaw = utils.smooth_yaw(cyaw)

    while not rospy.is_shutdown():
        pathPub.publish(my_path)

        xref, target_ind, dref = utils.calc_ref_trajectory(
            robot_state, cx, cy, cyaw, ck, sp, dl, dt, target_ind)

        #x0 = [robot_state.x, robot_state.y, robot_state.v, robot_state.yaw]  # current state
        #print(robot_state.get_current_meas_state())
        oa, ow, ox, oy, oyaw, ov = iterative_linear_mpc_control(
            xref, dref, oa, ow)

        if ow is not None:
            wi, ai = ow[0], oa[0]
                  
        # warm-up solver
        if True: #target_ind < 10:
            if abs(robot_state.v) < 0.05:
                if sp[target_ind]<0:
                    ai = -0.1
                else:
                    #print("Here")
                    ai = 0.1
        
        #apply the control signals
        cmd_command = make_twist_msg(ai, wi)
        controlPub.publish(cmd_command)

        
        rate.sleep()


if __name__ == '__main__':
    mpc_node()
    