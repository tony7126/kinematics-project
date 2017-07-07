#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *

def dh_func(alpha, a, q, d):
    return Matrix([[cos(q), -sin(q), 0, a],
            [sin(q) * cos(alpha), cos(q) * cos(alpha), -sin(alpha), -sin(alpha) * d],
            [sin(q) * sin(alpha), cos(q) * sin(alpha), cos(alpha), cos(alpha) * d],
            [0, 0, 0, 1]
           ])
def rot_x(q):
    R_x = Matrix([[ 1,              0,        0],
              [ 0,        cos(q), -sin(q)],
              [ 0,        sin(q),  cos(q)]])
    
    return R_x
    
def rot_y(q):              
    R_y = Matrix([[ cos(q),        0,  sin(q)],
              [       0,        1,        0],
              [-sin(q),        0,  cos(q)]])
    
    return R_y

def rot_z(q):    
    R_z = Matrix([[ cos(q), -sin(q),        0],
              [ sin(q),  cos(q),        0],
              [ 0,              0,        1]])
    
    return R_z
q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8', real=True)
d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8', real=True)
a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7', real=True)
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

origin = Matrix([[0], [0], [0]])
bottom_row = Matrix([[0, 0, 0, 1]])

s = {alpha0: 0, a0: 0, d1: 0.75,
    alpha1: -pi/2, a1: 0.35, d2: 0, q2: q2-pi/2,
    alpha2: 0, a2: 1.25, d3: 0,
    alpha3: -pi/2, a3: -0.054, d4: 1.50,
    alpha4: pi/2, a4: 0, d5: 0,
    alpha5: -pi/2, a5: 0, d6: 0,
    alpha6: 0, a6: 0, d7: .303, q7: 0}

u = atan2(-s[a3], s[d4])
v = sqrt(s[a3] ** 2 + s[d4]**2)

T0_1 = dh_func(alpha0, a0, q1, d1)
T0_1 = T0_1.subs(s)

T1_2 = dh_func(alpha1, a1, q2, d2)
T1_2 = T1_2.subs(s)


T2_3 = dh_func(alpha2, a2, q3, d3)
T2_3 = T2_3.subs(s)

T3_4 = dh_func(alpha3, a3, q4, d4)
T3_4 = T3_4.subs(s)

T4_5 = dh_func(alpha4, a4, q5, d5)
T4_5 = T4_5.subs(s)

T5_6 = dh_func(alpha5, a5, q6, d6)
T5_6 = T5_6.subs(s)

T6_G = dh_func(alpha6, a6, q7, d7)
T6_G = T6_G.subs(s)

T0_2 = simplify(T0_1 * T1_2)
T0_3 = simplify(T0_2 * T2_3)
T0_4 = simplify(T0_3 * T3_4)
T0_5 = simplify(T0_4 * T4_5)
T0_6 = simplify(T0_5 * T5_6)
T0_G = simplify(T0_6 * T6_G)

origin = Matrix([[0], [0], [0]])
R_corr_z = rot_z(-pi).row_join(origin).col_join(bottom_row)
R_corr_y = rot_y(pi/2).row_join(origin).col_join(bottom_row)
R_corr = simplify(R_corr_z * R_corr_y)
T_total = simplify(T0_G * R_corr)

T1_0 = dh_func(-alpha0, -a0, -q1, -d1)
T1_0 = T1_0.subs(s)

T1_0_shift = (rot_y(pi/2) * rot_z(pi/2)).row_join(Matrix([[0], [-a1], [0]])).col_join(bottom_row)
T1_0_shift = T1_0_shift.subs(s) * T1_0


T3_6 = simplify(T3_4 * T4_5 * T5_6 * T6_G * R_corr)
R3_6 = rot_z(-pi/2) * rot_y(-pi/2) * T3_6[0:3, 0:3]

R_corr_z1 = rot_z(pi)
R_corr_y1 = rot_y(-pi/2)

def predict_angle(last_side):
    return acos(((a2**2 + v**2 - last_side**2)/(2*a2*v))).subs(s)

def predict_angle2(last_side):
    return (acos(((last_side**2 + a2**2 - v**2)/(2*a2*last_side)))).subs(s)

def predict_angle3(last_side):
    return (acos(((last_side**2 + v**2 - a2**2)/(2*last_side*v)))).subs(s)

def get_R0_6(qx, qy, qz, qw):
    return Matrix([[1 - 2*qy**2 - 2*qz**2, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw],
    [2*qx*qy + 2*qz*qw, 1 - 2*qx**2 - 2*qz**2, 2*qy*qz - 2*qx*qw],
    [2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1 - 2*qx**2 - 2*qy**2]])

def to_wc(coord, rot):
    z_vec = (get_R0_6(*rot)* R_corr_z1 * R_corr_y1)[0:3,2]
    return Matrix([[coord[0] - s[d7] * z_vec[0]], [coord[1] - s[d7] * z_vec[1]], [coord[2] - s[d7] * z_vec[2]], [1]])

def get_theta_one(pos):
    first = atan2(pos[1], pos[0])
    if first > 0:
        return first, first - pi
    else:
        return first, first + pi

def get_theta2_pairs(shifted_vec):
    third_side = (sqrt(shifted_vec[1] ** 2 + shifted_vec[0]**2))
    theta3 = (pi/2-predict_angle(third_side))-u
    theta3 = -(theta3 + pi + 2* u)
    triangle_angle_2 = predict_angle2(third_side)
    part_one = atan2(shifted_vec[0], shifted_vec[1])

    theta2_poss_1 = pi/2 - (part_one + triangle_angle_2)
    theta3_poss_1 = (pi/2-predict_angle(third_side))-u
    theta2_poss_2 = pi/2 - (part_one - triangle_angle_2)
    theta3_poss_2 = -(theta3_poss_1 + pi + 2* u)
    return (theta2_poss_1, theta3_poss_1), (theta2_poss_2, theta3_poss_2)



def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        # Initialize service response
        joint_trajectory_list = []
        last_theta4 = 0
        last_theta5 = 0
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z
     
            quart = [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w]
            # Calculate joint angles using Geometric IK method
            wc_coord = to_wc([px,py,pz], quart) #get wrist center
            R0_6 = get_R0_6(*quart) #get Rrpy
            found = False
            theta_one_possibilities = get_theta_one(wc_coord)
            for theta1 in theta_one_possibilities:
                T1_shift_wc_coord = simplify(T1_0_shift * wc_coord).evalf(subs={q1:theta1})
                theta_2_3_pairs = get_theta2_pairs(T1_shift_wc_coord)
                for pair in theta_2_3_pairs:
                    theta2, theta3 = pair
                    R0_3 = T0_3[0:3, 0:3].subs({q1:theta1, q2:theta2, q3:theta3}) * rot_x(pi/2) * rot_y(pi/2)
                    R3_0 = Inverse(R0_3)
                    final_mat = R3_0 * R0_6 
                    theta5 = acos(final_mat[0,0])
                    
                    mult = -1 if theta5 < 0 else 1
                    theta4 = atan2(mult * -final_mat[1,0],final_mat[2,0])
                    test_ans = sin(theta5) * sin(theta4)
                    if not abs(test_ans - final_mat[1,0] / test_ans) < .002:
                        """
                        if abs(last_theta4 - (theta4 + pi)) < abs(last_theta4 - (theta4 - pi)):
                            theta4 += pi
                        else:
                            theta4 -= pi
                        """  
                        if theta4 > 0:
                            theta4 -= pi
                        else:
                            theta4 += pi
                        

                    last_theta4 = theta4
                    theta6 = atan2(final_mat[0,1], final_mat[0,2])
                    last_theta5 = theta5
                    if all([t.is_real for t in [theta4, theta5, theta6]]):
                        found = True
                        break
                if found:
                    break

            if not all([t.is_real for t in [theta4, theta5, theta6]]):
                print [theta1, theta2, theta3, theta4, theta5, theta6]
                print [px,py,pz]
                print quart
                raise Exception   
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
            print [theta1, theta2, theta3, theta4, theta5, theta6]
            joint_trajectory_point.positions = [theta1.evalf(), 
                                        theta2.evalf(), 
                                        theta3.evalf(), 
                                        theta4.evalf(), 
                                        theta5.evalf(), theta6.evalf()]
            joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
