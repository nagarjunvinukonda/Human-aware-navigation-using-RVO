
from math import ceil, floor, sqrt
import copy
import numpy

from math import cos, sin, tan, atan2, asin
from math import pi as PI


def RVO_update(X, V_des, V_current, ws_model):
    """ compute best velocity given the desired velocity, current velocity and workspace model"""


    ROB_RAD = ws_model['robot_radius'] + 0.1 # robot radius
    V_opt = list(V_current) # velocities of robots

    # Multi agents
    # For each robot
    for i in range(len(X)):
        # Current robot
        vA = [V_current[i][0], V_current[i][1]] # current velocity
        pA = [X[i][0], X[i][1]] # current position
        RVO_BA_all = [] # velocity cones

        # For each agent other than current robot, build velocity cone
        for j in range(len(X)):
            if i!=j:

                vB = [V_current[j][0], V_current[j][1]] # vel of the other agent
                pB = [X[j][0], X[j][1]] # pos of the other agent

                # use RVO
                # next position if take 1/2(Va + Vb)
                transl_vB_vA = [pA[0] + 0.5*(vB[0]+vA[0]), pA[1] + 0.5*(vB[1]+vA[1])]
                
                dist_BA = distance(pA, pB)
                theta_BA = atan2(pB[1]-pA[1], pB[0]-pA[0])

                # set dist between (2 * robot_rad, inf)
                if 2 * ROB_RAD > dist_BA:
                    dist_BA = 2 * ROB_RAD

                # Compute velocity cone angles (left, right)
                theta_BAort = asin(2*ROB_RAD/dist_BA)
                theta_ort_left = theta_BA + theta_BAort
                theta_ort_right = theta_BA - theta_BAort

                bound_left = [cos(theta_ort_left), sin(theta_ort_left)]
                bound_right = [cos(theta_ort_right), sin(theta_ort_right)]

                # use HRVO
                # dist_dif = distance([0.5*(vB[0]-vA[0]),0.5*(vB[1]-vA[1])],[0,0])
                # transl_vB_vA = [pA[0]+vB[0]+cos(theta_ort_left)*dist_dif, pA[1]+vB[1]+sin(theta_ort_left)*dist_dif]
                RVO_BA = [transl_vB_vA, bound_left, bound_right, dist_BA, 2*ROB_RAD]
                RVO_BA_all.append(RVO_BA)

        # For each obstacle, build velocity cone
        for hole in ws_model['circular_obstacles']:

            vB = [0, 0] # vel = 0
            pB = hole[0:2] # hole = [x, y, rad]
            
            # next position if take 1/2(Va + Vb)
            transl_vB_vA = [pA[0] + vB[0], pA[1] + vB[1]]

            dist_BA = distance(pA, pB)
            theta_BA = atan2(pB[1]-pA[1], pB[0]-pA[0])

            # over-approximation of square to circular
            OVER_APPROX_C2S = 1.5
            rad = hole[2]*OVER_APPROX_C2S
            # set dist between (obstacle_rad + robot_rad, inf)
            if (rad + ROB_RAD) > dist_BA:
                dist_BA = rad + ROB_RAD
            
            # Compute velocity cone angles (left, right)
            theta_BAort = asin((rad + ROB_RAD)/dist_BA)
            theta_ort_left = theta_BA + theta_BAort
            theta_ort_right = theta_BA - theta_BAort

            bound_left = [cos(theta_ort_left), sin(theta_ort_left)]
            bound_right = [cos(theta_ort_right), sin(theta_ort_right)]

            RVO_BA = [transl_vB_vA, bound_left, bound_right, dist_BA, rad + ROB_RAD]
            RVO_BA_all.append(RVO_BA)

        # Find optimal velocity given all the velocity cones
        vA_post = intersect(pA, V_des[i], RVO_BA_all)
        V_opt[i] = vA_post[:]
    return V_opt


def intersect(pA, vA, RVO_BA_all):
    # print '----------------------------------------'
    # print 'Start intersection test'

    """ It checks for all the intersecting RVO cones there lies a suitable velocity outside or not. If not avaiable it selects V inside RVO, which has min penality """

    norm_v = distance(vA, [0, 0])
    suitable_V = []
    unsuitable_V = []
    
    # A random theta
    for theta in numpy.arange(0, 2*PI, 0.1):
        # A random vector constant for desired velocity
        for rad in numpy.arange(0.02, norm_v+0.02, norm_v/5.0):

            # This is for checking any random velocity vector with start point as desired velocity is in RVO cone or not
            # A desired velocity vector with random direction both x and y            
            new_v = [rad*cos(theta), rad*sin(theta)]
            suit = True
            for RVO_BA in RVO_BA_all:
                #shifted VO cone apex position
                p_0 = RVO_BA[0]
                left = RVO_BA[1]
                right = RVO_BA[2]
                # dif is considering position coordinates of velocity vectors(desired + current) inside new shifted VO cone
                dif = [new_v[0]+pA[0]-p_0[0], new_v[1]+pA[1]-p_0[1]]
                # theta_dif is angle formed by new vectors
                theta_dif = atan2(dif[1], dif[0])
                # theta_ort_right
                theta_right = atan2(right[1], right[0])
                # theta_ort_left
                theta_left = atan2(left[1], left[0])
                # if the velocity vector lies inside the VO cone its not suitable velocity
                if in_between(theta_right, theta_dif, theta_left):
                    suit = False
                    break
            if suit:
                suitable_V.append(new_v)
            else:
                unsuitable_V.append(new_v)                
    
    # new_V here is desired or preffered V (this is different from previous new_V)
    new_v = vA[:]
    suit = True

    # This is for checking only the absolute value of desired velocity vector lies in RVO cone or not
    for RVO_BA in RVO_BA_all:                
        p_0 = RVO_BA[0]
        left = RVO_BA[1]
        right = RVO_BA[2]
        dif = [new_v[0]+pA[0]-p_0[0], new_v[1]+pA[1]-p_0[1]]
        theta_dif = atan2(dif[1], dif[0])
        theta_right = atan2(right[1], right[0])
        theta_left = atan2(left[1], left[0])
        if in_between(theta_right, theta_dif, theta_left):
            suit = False
            break
    if suit:
        suitable_V.append(new_v)
    else:
        unsuitable_V.append(new_v)
    #----------------------        
    if suitable_V:
        # print 'Suitable found'
        # selecting a velocity vector among suitable velocities closest to desired velocity(VA)
        vA_post = min(suitable_V, key = lambda v: distance(v, vA))
        new_v = vA_post[:]
        for RVO_BA in RVO_BA_all:
            p_0 = RVO_BA[0]
            left = RVO_BA[1]
            right = RVO_BA[2]
            dif = [new_v[0]+pA[0]-p_0[0], new_v[1]+pA[1]-p_0[1]]
            theta_dif = atan2(dif[1], dif[0])
            theta_right = atan2(right[1], right[0])
            theta_left = atan2(left[1], left[0])
    else:
        # Here we are penalizing the unsuitable V tthat are inside RVO. For more information on formula check "Reciprocal Velocity Obstacles for Real-Time Multi-Agent Navigation" page 5.

        # if No suitable velocity is avilable in combined RVO we penalise and select the velocity with min penality.
        # print 'Suitable not found'
        # tc_V is expected time to collision with prefered velocity Vi
        tc_V = dict()
        for unsuit_v in unsuitable_V:
            # create an empty list of tc_V
            tc_V[tuple(unsuit_v)] = 0
            tc = []
            for RVO_BA in RVO_BA_all:
                p_0 = RVO_BA[0]
                left = RVO_BA[1]
                right = RVO_BA[2]
                # dist_BA
                dist = RVO_BA[3]
                rad = RVO_BA[4]
                dif = [unsuit_v[0]+pA[0]-p_0[0], unsuit_v[1]+pA[1]-p_0[1]]
                theta_dif = atan2(dif[1], dif[0])
                theta_right = atan2(right[1], right[0])
                theta_left = atan2(left[1], left[0])
                if in_between(theta_right, theta_dif, theta_left):
                    small_theta = abs(theta_dif-0.5*(theta_left+theta_right))
                    if abs(dist*sin(small_theta)) >= rad:
                        rad = abs(dist*sin(small_theta))
                    big_theta = asin(abs(dist*sin(small_theta))/rad)
                    dist_tg = abs(dist*cos(small_theta))-abs(rad*cos(big_theta))
                    if dist_tg < 0:
                        dist_tg = 0                    
                    tc_v = dist_tg/distance(dif, [0,0])
                    tc.append(tc_v)
            # For the combined reciprocal velocity obstacle, the expected time to collision is the minimum of all expected times to collision(tc) with respect to the individual other agents and obstacles, and infinity when there is no collision.       
            
            tc_V[tuple(unsuit_v)] = min(tc)+0.001

        # Minimize
        # 1, penality
        # 2, difference from desired velocity

        # WT is Omega(Angular velcity). This can vary among the agents to reflect differences in aggressiveness and sluggishness.
        WT = 0.2

        # Penality formula: (Available in paper)
        vA_post = min(unsuitable_V, key = lambda v: ((WT/tc_V[tuple(v)]) + distance(v, vA)))
        
    return vA_post 


def in_between(theta_right, theta_dif, theta_left):
    """ Checks if the theta_diff lies between theta_right and theta_left"""

    if abs(theta_right - theta_left) <= PI:
        if theta_right <= theta_dif <= theta_left:
            return True
        else:
            return False
    else:
        if (theta_left <0) and (theta_right >0):
            theta_left += 2*PI
            if theta_dif < 0:
                theta_dif += 2*PI
            if theta_right <= theta_dif <= theta_left:
                return True
            else:
                return False
        if (theta_left >0) and (theta_right <0):
            theta_right += 2*PI
            if theta_dif < 0:
                theta_dif += 2*PI
            if theta_left <= theta_dif <= theta_right:
                return True
            else:
                return False


def compute_V_des(X, goal, V_max):
    """ Computing desired velocity from start to goal"""

    V_des = []
    # For every agent
    for i in range(len(X)):
        # vector - distance to goal
        dif_x = [goal[i][k] - X[i][k] for k in range(2)]
        # scaler - distance to goal
        norm = distance(dif_x, [0, 0])

        # vector - desired velocity pointing towards the goal
        norm_dif_x = [dif_x[k]*V_max[k]/norm for k in range(2)]
        V_des.append(norm_dif_x[:])

        # if reach the goal, v = 0 
        if reach(X[i], goal[i], 0.1):
            V_des[i][0] = 0
            V_des[i][1] = 0
    return V_des


def reach(p1, p2, bound=0.5):
    if distance(p1,p2)< bound:
        return True
    else:
        return False


def distance(pose1, pose2):
    """ compute Euclidean distance for 2D """
    return sqrt( (pose1[0] - pose2[0])**2 + (pose1[1] - pose2[1])**2 ) + 0.001
