#!/usr/bin/env python

# Copyright (C) 2017 Udacity Inc.
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



def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        # joint_trajectory_point = JointTrajectoryPoint()
        ### Your FK code here
        # Create symbols
        alpha0,alpha1,alpha2,alpha3,alpha4,alpha5,alpha6 = symbols('alpha0:7')
        a0,a1,a2,a3,a4,a5,a6 = symbols('a0:7')
        q1,q2,q3,q4,q5,q6,q7 = symbols('q1:8')
        d0,d1,d2,d3,d4,d5,d6 = symbols('d0:7')
	#
	#
	# Create Modified DH parameters

        DH_table = {alpha0:0 ,alpha1:-pi/2 ,alpha2:0 ,alpha3:-pi/2 ,alpha4:0 ,alpha5:-pi/2 ,alpha6:0 ,
                    a0:0 ,a1:0.35 ,a2:1.25 ,a3:-0.054 ,a4:0 ,a5:0 ,a6:0 ,
                    q1: q1,q2:-pi/2.+q2 ,q3:q3 ,q4:q4 ,q5:q5 ,q6:q6 ,q7:q7 ,
                    d0:0.75 ,d1:0 ,d2:0 ,d3:1.5 ,d4:0 ,d5:0 ,d6:0.303 }
	#
	#
	# Define Modified DH Transformation matrix
	#
	#
        
	# Create individual transformation matrices
	#
        t0_1 = tf_matrix(alpha0, a0, q1, d0).subs(DH_table)
        t1_2 = tf_matrix(alpha1, a1, q2, d1).subs(DH_table)
        t2_3 = tf_matrix(alpha2, a2, q3, d2).subs(DH_table)
        t0_3 = simplify(t0_1*t1_2*t2_3)
        

        rot_corrected = rot_z(radians(180))* rot_y(radians(-90))

  
        

	#
	# Extract rotation matrices from the transformation matrices
	#




        ###

        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

	    # Extract end-effector position and orientation from request
	    # px,py,pz = end-effector position
	    # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])

            ### Your IK code here
	    # Compensate for rotation discrepancy between DH parameters and Gazebo
	    #End effector position
            EE = Matrix([[px],
                         [py],
                         [pz]])
            
            R_x = rot_x(roll)
            R_y = rot_y(pitch)
            R_z = rot_z(yaw)

            r_EE = R_x * R_y * R_z * rot_corrected


            wc = EE - (0.303) * r_EE[:,2]

	    #
	    # Calculate joint angles<theta> using Geometric IK method
	    #
	    #
            side_a = 1.501
            side_b = sqrt((sqrt((wc[0]**2)+(wc[1]**2)) - 0.35)**2 + ((wc[2]-0.75)**2))
            side_c = 1.25

            # to find the inner angles of the triangle
            c_a = (side_b**2 + side_c**2 - side_a**2)/(2 * side_b * side_c)
            a = atan2(sqrt(1 - c_a**2), c_a)

            c_b = (side_a**2 + side_c**2 - side_b**2)/(2 * side_a * side_c)
            b = atan2(sqrt(1 - c_b**2), c_b)


            theta1 = atan2(wc[1],wc[0])
            theta2 = (pi/2) - a - atan2(wc[2]-0.75, sqrt((wc[0]**2)+(wc[1]**2))-0.35)
            theta3 = (pi/2) - (b+0.036)

            #rotation matrix fro transformation matrices
            r0_3 = t0_3[0:3, 0:3]
            r0_3 = r0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})

            #rotation matrix r3_6 = transpose of r0_3 * r_EE
            r3_6 = r0_3.transpose()*r_EE

            theta4 = atan2(r3_6[2,2],-r3_6[0,2])
            theta5 = atan2(sqrt(r3_6[0,2]**2 + r3_6[2,2]**2), r3_6[1,2])
            theta6 = atan2(-r3_6[1,1],r3_6[1,0])
	    
	    if x == 0:
                theta4_p = theta4

            while (theta4 - theta4_p > pi):
                theta4 = theta4 - 2 * pi
            while (theta4 - theta4_p < -pi):
                theta4 = theta4 + 2 * pi

            theta4_p = theta4


            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
            joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
            joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)

def rot_x(roll):
    R_x = Matrix([[ 1,         0,          0],
                  [ 0, cos(roll), -sin(roll)],
                  [ 0, sin(roll),  cos(roll)]]) #roll
    return R_x

def rot_y(pitch):
    R_y = Matrix([[  cos(pitch), 0,   sin(pitch)],
                  [       0,     1,            0],
                  [ -sin(pitch), 0,   cos(pitch)]]) #pitch
    return R_y
    
def rot_z(yaw):        
    R_z = Matrix([[ cos(yaw), 0,  -sin(yaw)],
                  [ sin(yaw), 0,   cos(yaw)],
                  [        0, 1,         0]]) #yaw
    return R_z

def tf_matrix(alpha,a,q,d):
    tf = Matrix([[           cos(q),           -sin(q),          0,            a],
                 [sin(q)*cos(alpha), cos(alpha)*cos(q), sin(alpha), sin(alpha)*d],
                 [sin(q)*sin(alpha), cos(q)*sin(alpha), cos(alpha), cos(alpha)*d],
                 [                0,                 0,          0,            1]])
    return tf


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
