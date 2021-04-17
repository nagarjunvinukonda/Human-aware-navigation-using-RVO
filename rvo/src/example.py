#!/usr/bin/env python2
import sys
import rospy
import numpy as np
import decimal

from RVO import RVO_update, reach, compute_V_des, reach, distance
from vis import visualize_traj_dynamic
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import GetModelState
from tf.transformations import euler_from_quaternion


# def OdomData(data):
#     return data

#------------------------------
def rvo_start():
    #define workspace model
    ws_model = dict()
    #robot radius
    ws_model['robot_radius'] = 0.2
    #circular obstacles, format [x,y,rad]
    # no obstacles
    # ws_model['circular_obstacles'] = [[0.0, 0.0, 1.0]]
    ws_model['circular_obstacles'] = []
    # with obstacles
    # ws_model['circular_obstacles'] = [[-0.3, 2.5, 0.3], [1.5, 2.5, 0.3], [3.3, 2.5, 0.3], [5.1, 2.5, 0.3]]
    #rectangular boundary, format [x,y,width/2,heigth/2]
    ws_model['boundary'] = [] 

    
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()
    

    # rospy.spin()



    # vel_msg.linear.y = 0
    # vel_msg.linear.z = 0
    # vel_msg.angular.x = 0
    # vel_msg.angular.y = 0
    # vel_msg.angular.z = 0

    #------------------------------
    #initialization for robot 
    # position of [x,y]

    # X = [[-0.5+1.0*i, 0.0] for i in range(7)] + [[-0.5+1.0*i, 5.0] for i in range(7)]

    # odom_sub = rospy.Subscriber('/odom', Odometry, OdomData)
    # odom_data = OdomData(Odometry)

    X = [[-5.2,-2.5]]

    # velocity of [vx,vy]
    # V = [[0,0] for i in range(len(X))]

    V = [[0,0]]

    # maximal velocity norm
    # V_max = [1.0 for i in range(len(X))]

    V_max = [1,1]

    # goal of [x,y]
    # goal = [[5.5-1.0*i, 5.0] for i in range(7)] + [[5.5-1.0*i, 0.0] for i in range(7)]

    goal = [[5.0,0.0]]

    #------------------------------
    #simulation setup
    # total simulation time (s)
    total_time = 15
    # simulation step
    step = 0.01
    
    def float_range(start, stop, step):
        while start < stop:
            yield float(start)
            start += decimal.Decimal(step)

    #------------------------------
    #simulation starts
    t = 0
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rospy.loginfo("I hope its working")
        # while t*step < total_time:
        # while distance(X[0],goal[0])!=0.1:
        # while round(distance(X[0],goal[0]),2) not in np.arange(0.01,0.3,0.01):
        t0 = rospy.Time.now().to_sec()
        

        last_time = rospy.Time.now().to_sec()
        while round(distance(X[0],goal[0]),2) not in list(float_range(0, 0.2, '0.01')):
            # compute desired vel to goal
            V_des = compute_V_des(X, goal, V_max)
            # compute the optimal vel to avoid collision
            V = RVO_update(X, V_des, V, ws_model)

            
            # rospy.sleep(2.)
            print("last_time %.4f"%(last_time))
            current_time = rospy.Time.now().to_sec()
            
            time_diff = current_time-last_time
            last_time = current_time
            print("time_diff %.4f"%(time_diff))

            print("current_time %.4f"%(current_time))

            desired_theta = np.arctan2(V[0][1],V[0][0])

            print("desired_theta %.4f"%(desired_theta))


            yaw= desired_theta/(time_diff)
            print("yaw_up %.4f"%(yaw))

            yaw = round(yaw,2)
            print("yaw_down %.4f"%(yaw))

            # #compute odometry in a typical way given the velocities of the robot

            # dt = (current_time.secs - last_time.secs)
            # delta_x = (vx * cos(th)) * dt
            # delta_y = (vx * sin(th)) * dt
            # delta_th = vth * dt

            # print(odom_data.pose.pose.position.x)
            linear_vel=np.sqrt((V[0][0])**2+(V[0][1])**2)
            linear_vel = round(linear_vel,3)
            vel_msg.linear.x = linear_vel
            # vel_msg.linear.y = V[0][1]
            vel_msg.angular.z = yaw
            # print(vel_msg)

            print("Vx: %.2f         Vy: %.2f"%(vel_msg.linear.x, vel_msg.linear.y ))
            # print('len v: %d'%(len(V)))
            # print(V)

            t1=rospy.Time.now().to_sec()
            # print(t1-t0)
            # update position
            # for i in range(len(X)):
            #     X[i][0] += V[i][0]*step
            #     X[i][1] += V[i][1]*step

            rospy.wait_for_service('/gazebo/get_model_state')
            try:
                get_robot_pose = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
                data = get_robot_pose("turtlebot3_waffle", "ground_plane")
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)

            base_position = data.pose.position

            print("X:  %.2f  $$$$$$   Y: %.2f" %(base_position.x,base_position.y))
            X[0][0]=base_position.x
            X[0][1]=base_position.y

            # p = np.array([base_position.x, base_position.y])
            # base_orientation = data.pose.orientation
            # quaternion = [base_orientation.x,
            #             base_orientation.y, 
            #             base_orientation.z, 
            #             base_orientation.w]
            # euler = euler_from_quaternion(quaternion)
            # current_yaw = euler[2]
            

            print("Pos_X: %.2f   ####   Pos_Y: %.2f" %(X[0][0],X[0][1]))
            velocity_publisher.publish(vel_msg)
            rate.sleep()
            #----------------------------------------
            # # visualization
            # if t%10 == 0:
            #     visualize_traj_dynamic(ws_model, X, V, goal, time=t*step, name='data/snap%s.png'%str(t/10))
            #     #visualize_traj_dynamic(ws_model, X, V, goal, time=t*step, name='data/snap%s.png'%str(t/10))

            t += 1

        vel_msg.linear.x = 0.0
        # vel_msg.linear.y = 0.0
        vel_msg.angular.z = 0.0
        # print(vel_msg)
        velocity_publisher.publish(vel_msg)
        # rate.sleep()
    
if __name__ == '__main__':
    rospy.init_node('RVO_Bot')
    try:
         #Testing our function
        rvo_start()
    except rospy.ROSInterruptException: 
        rospy.loginfo('Shutting down')