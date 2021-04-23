#!/usr/bin/env python2
import sys
import rospy
import numpy as np

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import GetModelState
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from RVO import RVO_update, reach, compute_V_des, reach, distance


def twist_to_xyyaw(data):
        position = data.pose.position
        x = position.x
        y = position.y
        orientation = data.pose.orientation
        quaternion = [orientation.x,
                      orientation.y, 
                      orientation.z, 
                      orientation.w]
        euler = euler_from_quaternion(quaternion)
        yaw = euler[2]

        return x, y, yaw


def get_robot_pose(robot_name):
    ''' Ground truth
    '''
    rospy.wait_for_service('/gazebo/get_model_state')
    try:
        robot_state_service = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        robot_state_res = robot_state_service(robot_name, "")
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

    return twist_to_xyyaw(robot_state_res)


def map_xy_to_xz(v_robot, t_diff, k):
    if k * t_diff == 0:
        return 0, 0
    # Ori.z
    desired_theta = np.arctan2(v_robot[1], v_robot[0])
    omega = desired_theta / (k * (t_diff))
    # Vel.x
    vel = np.sqrt((v_robot[0])**2 + (v_robot[1])**2)

    return vel, omega


# Another way to encounter while loop:
# def float_range(start, stop, step):
#     while start < stop:
#         yield float(start)
#         start += decimal.Decimal(step)

def rvo_test():
    # ROS node
    rospy.init_node('RVO_Bot')
    rate = rospy.Rate(10)
    velocity_publisher = rospy.Publisher('gopher_1/base_controller/cmd_vel', Twist, queue_size=1)
    visual_publisher = rospy.Publisher('gopher_1/rvo', PoseStamped, queue_size=1)
    vel_msg = Twist()
    
    # RVO setting
    # define workspace model
    ws_model = dict()
    # robot radius
    ws_model['robot_radius'] = 0.2
    # circular obstacles, format [x, y, rad]
    ws_model['circular_obstacles'] = []

    # Robot settings
    # initialization
    X = [[0, 0]]
    V = [[0, 0]]
    # robot pos & goal
    goal = [[5.0, -1.0]]
    # robot initial vel & max velocity
    V_max = [1, 1]
    
    # Simulation starts
    last_time = rospy.Time.now().to_sec()
    while not rospy.is_shutdown():
        rospy.loginfo("Start simulation")
        
        # Another way to encounter while loop:
        # while round(distance(X[0],goal[0]),2) not in list(float_range(0, 0.2, '0.01')):
        while not (distance(X[0], goal[0]) < 0.5):
            # Get the latest position of the robot
            x, y, yaw = get_robot_pose("gopher_1")
            X = [[x, y]]

            # Run RVO
            # compute desired vel to goal
            V_des = compute_V_des(X, goal, V_max)

            # compute the optimal vel to avoid collision
            V = RVO_update(X, V_des, V, ws_model)

            # Map vx, vy to v_pos.x, v_ori.z
            # delta_t
            current_time = rospy.Time.now().to_sec()
            time_diff = current_time - last_time
            last_time = current_time
            # w.r.t. robot

            # We are chaning cartisian coordinates of simulation velocities into polar coordinates
            v_robot = [ np.cos(yaw)*V[0][0] + np.sin(yaw)*V[0][1], 
                       -np.sin(yaw)*V[0][0] + np.cos(yaw)*V[0][1]] 
            # to v.x and o.z
            vel, omega = map_xy_to_xz(v_robot, time_diff, 3)

            # Publish to robot
            vel_msg.linear.x = float(vel)
            vel_msg.angular.z = float(omega)
            velocity_publisher.publish(vel_msg)

            # Visualize RVO result
            vis = PoseStamped()
            vis.header.stamp = rospy.Time.now()
            vis.header.frame_id = "map"
            vis.pose.position.x = x
            vis.pose.position.y = y
            [vis.pose.orientation.x, vis.pose.orientation.y, vis.pose.orientation.z,  
            vis.pose.orientation.w] = quaternion_from_euler(0, 0, np.arctan2(V[0][1], V[0][0]))
            visual_publisher.publish(vis)
            
            print("Pos_X: %.2f, Pos_Y: %.2f" %(X[0][0], X[0][1]))
            print("Vx: %.02f, Vy: %.2f"%(v_robot[0], v_robot[1]))
            print("time_diff %.4f"%(time_diff))
            print("Vel: %.2f, Omega: %.2f"%(vel, omega))
            # print

            rate.sleep()
            
        # Stop robot
        velocity_publisher.publish(Twist())
        break

    
if __name__ == '__main__':
    try:
        rvo_test()
    except rospy.ROSInterruptException: 
        rospy.loginfo('Shutting down')