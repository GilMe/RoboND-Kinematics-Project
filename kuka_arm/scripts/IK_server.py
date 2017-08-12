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


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

     	# #############
	    # Compute tranform equations 
	    # These are general eqations that will be used for all the inverse 
        # kinematic caculations

        # Define DH param symbols

        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

                
        # Joint angle symbols
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8') #theta_i

          
            # Modified DH params
        # Modified DH params
        s = {alpha0:     0, a0:      0, d1:  0.75,  q1:       q1,
            alpha1: -pi/2., a1:   0.35, d2:     0,  q2: q2-pi/2.,
            alpha2:      0, a2:   1.25, d3:     0,  q3:       q3,
         	alpha3: -pi/2., a3: -0.054, d4:  1.50,  q4:       q4,
         	alpha4:  pi/2., a4:      0, d5:     0,  q5:       q5,
            alpha5: -pi/2., a5:      0, d6:     0,  q6:       q6,
         	alpha6:      0, a6:      0, d7: 0.303,  q7:        0}

                
            # Define Modified DH Transformation matrix


	    # base_link to link1
        T0_1 = Matrix([[             cos(q1),           -sin(q1),            0,               a0],
               	       [ sin(q1)*cos(alpha0), cos(q1)*cos(alpha0), -sin(alpha0), -sin(alpha0)*d1],
                   	       [ sin(q1)*sin(alpha0), cos(q1)*sin(alpha0),  cos(alpha0),  cos(alpha0)*d1],
                   	       [                   0,                  0,            0,                1]])

        # link1 to link2
        T1_2 = Matrix([[             cos(q2),           -sin(q2),            0,               a1],
                	       [ sin(q2)*cos(alpha1), cos(q2)*cos(alpha1), -sin(alpha1), -sin(alpha1)*d2],
                   	       [ sin(q2)*sin(alpha1), cos(q2)*sin(alpha1),  cos(alpha1),  cos(alpha1)*d2],
                   	       [                   0,                  0,            0,                1]])
     
        T2_3 = Matrix([[             cos(q3),           -sin(q3),            0,               a2],
                	       [ sin(q3)*cos(alpha2), cos(q3)*cos(alpha2), -sin(alpha2), -sin(alpha2)*d3],
                   	       [ sin(q3)*sin(alpha2), cos(q3)*sin(alpha2),  cos(alpha2),  cos(alpha2)*d3],
                   	       [                   0,                  0,            0,                1]])

        T0_1 = T0_1.subs(s)
        T1_2 = T1_2.subs(s)
        T2_3 = T2_3.subs(s)

        
        T0_2 = T0_1 * T1_2

        T0_3 = T0_1 * T1_2 * T2_3
     


	
	    ## End of transform equations calculations
	    ######

        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

            
            # INVERSE POSITION part of the problem: 
            # This part finding the theta1-theta3 for the wrist center  

            # px,py,pz = end-effector position
            # roll, pitch, yaw = end-effector orientation

            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                 req.poses[x].orientation.z, req.poses[x].orientation.w])

            
            r, p, y = symbols('r p y')
            
            # ROLL    
            ROT_x = Matrix([[ 1,      0,       0],
                            [ 0, cos(r), -sin(r)],
                            [ 0, sin(r),  cos(r)]]) 

            # PITCH    
            ROT_y = Matrix([[  cos(p), 0,  sin(p)],
                            [       0, 1,       0],
                            [ -sin(p), 0,  cos(p)]]) 

            # YAW
            ROT_z = Matrix([[ cos(y), -sin(y), 0],
                            [ sin(y),  cos(y), 0],
                            [       0,      0, 1]]) 
            
            ROT_EE = ROT_z * ROT_y * ROT_x
            
            ROT_Error = ROT_z.subs(y, radians(180)) * ROT_y.subs(p, radians(-90))

            ROT_EE = ROT_EE * ROT_Error
            ROT_EE = ROT_EE.subs({'r': roll, 'p': pitch, 'y': yaw})
            

        #    # Calculate joint angles using Geometric IK method
        #    # Calculating n -the z unit vector from the given orientation (roll, pitch, yaw)
            nx = ROT_EE[0,2]
            ny = ROT_EE[1,2]
            nz = ROT_EE[2,2]

            # first calculate the wrist center (wc) location
            wx = (px - d7*nx).subs(s)
            wy = (py - d7*ny).subs(s)
            wz = (pz - d7*nz).subs(s)



            my_wc = [wx, wy, wz]    

             # Now calculate q1
            theta1 = atan2(wy, wx)



             # Use q1 for to find origin of joint2
            base_to_j2 = T0_2.subs({q1: theta1})
            
            # The x, y, z values of the origin of joint 2
            j2_x = base_to_j2[0,3]
            j2_y = base_to_j2[1,3]
            j2_z = base_to_j2[2,3]

            # find the lengths of the sides of the triangle and angles
            A = (sqrt(a3**2 + d4**2)).subs(s)
            B = sqrt((wx-j2_x)**2 + (wy-j2_y)**2 + (wz-j2_z)**2)
            C =  a2.subs(s)

            angle_a = acos((B**2 + C**2 -A**2)/(2*B*C))
            angle_b = acos((A**2 + C**2 -B**2)/(2*A*C))
            angle_c = acos((A**2 + B**2 -C**2)/(2*A*B))

            alpha2 = atan2(wz-j2_z, sqrt((wx-j2_x)**2 + (wy-j2_y)**2))
            theta3_const = atan2(-a3,d4).subs(s)

            #calculate the thetas

            theta2 = (pi/2 - angle_a - alpha2).evalf()
            theta3 = (pi/2 - angle_b - theta3_const).evalf()

            # INVERSE ORIENTATION part of the problem
            # This part finding the theta4-theta6 to get the correct orientation of the end effector

            # Find the rotation 0-3
            R0_3 = T0_3[0:3, 0:3]
            R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})

            # Find the rotation 3-6
            R3_6 = R0_3.inv("LU") * ROT_EE    

            theta4 = atan2(R3_6[2,2], -R3_6[0,2]).evalf() 
            theta5 = atan2(sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2]*R3_6[2,2]), R3_6[1,2]).evalf() 
            theta6 = atan2(-R3_6[1,1],R3_6[1,0]).evalf()
           
            
#            theta1 = theta1.evalf()
#            theta2 = theta2.evalf()
#            theta3 = theta3.evalf()
#            theta4 = theta4.evalf() 
#            theta5 = theta5.evalf()
#            theta6 = theta6.evalf() 


    #            #print("inter =", x, "wx=", wx,"wy=", wy, "wz=", wz)
    #            print("inter =", x, "A =", A, "B=", B, "C=", C)
    #            print("wx=", wx,"wy=", wy, "wz=", wz, "j2_x=", j2_x,"j2_y=", j2_y, "j2_z=", j2_z)            
    #            print("angle_a=", angle_a, "angle_b=", angle_b, "angle_c=", angle_c, "alpha2=", alpha2)  
    #        print("theta4 =", theta4, "theta5 =", theta5, "theta6 =", theta6)
    #            

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
            joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
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
