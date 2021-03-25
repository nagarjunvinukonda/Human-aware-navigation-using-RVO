#!/usr/bin/env python
import rospy
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

import time


class SocialRobot():

    def __init__(self):
        # variables
        self.x = 0
        self.y = 0
        self.yaw = 0
        
        # instantiate the node
        rospy.init_node('social_robot')

        # subscribe to 
        self.odom_sub = rospy.Subscriber('base_controller/odom', Odometry, self.odom_callback)

        # publish to robot input
        self.cmd_vel_pub = rospy.Publisher('base_controller/cmd_vel', Twist, queue_size=1)


    def pose_to_xyyaw(self, data):
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

    
    def odom_callback(self, data):
        ''' Localization estimation
        '''
        pose = data.pose
        x, y, yaw = self.pose_to_xyyaw(pose)

        cmd_vel = Twist()
        cmd_vel.linear.x = 0
        cmd_vel.linear.y = 0
        cmd_vel.angular.z = 0
        self.cmd_vel_pub.publish(cmd_vel)


    def get_robot_pose(self):
        ''' Ground truth
        '''
        rospy.wait_for_service('/gazebo/get_model_state')
        try:
            state_service = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            robot_state_res = robot_state_service("gopher_1", "")
            robot_pose = robot_state_res.pose
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

        return self.pose_to_xyyaw(robot_pose)

        
    def get_human_pose(self):
        ''' Ground truth
        '''
        rospy.wait_for_service('/gazebo/get_model_state')
        try:
            state_service = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            human_state_res = robot_state_service("actor", "")
            human_pose = robot_state_res.pose
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

        return self.pose_to_xyyaw(human_pose)
        

if __name__ == "__main__":
    robot = SocialRobot()

    # rospy.spin()
    r = rospy.Rate(30)
    while not rospy.is_shutdown():
        r.sleep()
