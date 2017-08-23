## Project: Kinematics Pick & Place
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---


**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 


[//]: # (Image References)

[image1]: ./misc_images/misc1.png
[image2]: ./misc_images/misc3.png
[image3]: ./misc_images/misc2.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

We will start the forward kinematic analysis by analyzing the urdf file in order to get the robot dimension and get the joint locations that will help us derive the DH parameters. The URDF reference frame is different than the DH reference frame. The axis are the joint axis. Here is a schematic that shows the joint axis locations and orientations


The urdf file provides us with translations and rotations from one joint origin to another. The following table shows sums up these dimensions.

Joint Name | Parent Link | Child Link | x (m) | y (m) | z (m) | roll | pitch | yaw
--- | --- | --- | --- | --- | --- | --- | --- | 
joint_1 | base_link | link_1 | 0 | 0 | 0.33 | 0 | 0 | 0 |
joint_2 | link_1 | link_2 | 0.35 | 0 | 0.42 | 0 | 0 | 0 |
joint_3 | link_2 | link_3 | 0 | 0 | 1.25 | 0 | 0 | 0 |
joint_4 | link_3 | link_4 | 0.96 | 0 | -0.054 | 0 | 0 | 0 |
joint_5 | link_4 | link_5 | 0.54 | 0 | 0 | 0 | 0 | 0 |
joint_6 | link_5 | link_6 | 0.193 | 0 | 0 | 0 | 0 | 0 |
gripper_joint | link_6 | gripper_link | 0.11 | 0 | 0 | 0 | 0 | 0 |
right_gripper_finger_joint | gripper_link | right_gripper_finger_link | 0.15 | 0 | 0 | 0 | 0 | 0 |
left_gripper_finger_joint | gripper_link | left_gripper_finger_link | 0.15 | 0 | 0 | 0 | 0 | 0 |


We will now use these values to derive the DH parameters. Before we start we have find and the define the axis that we will be using. This can be seen in the next figure:
 
 ![alt text][image1]
 
Using information we collected from the urdf file the DH paramenters are presented in the following table:

i | $$\alpha_{i-1}$$  | $$a_{i-1}$$ | $$d_i$$ | $$ \theta_i $$ | 
--- | --- | --- | --- | 
$$ T^0_1 $$ | 0 | 0 | 0.75 | $$ \theta_1 $$ |
$$ T^1_2 $$ | $$-\pi/2$$ | 0.35 | 0 | $$\theta_2-\pi/2$$ |
$$ T^2_3 $$ | 0 | 1.25 | 0 | $$ \theta_3 $$ |
$$ T^3_4 $$ | $$-\pi/2$$ | -0.054 | 1.5 | $$ \theta_4 $$ |
$$ T^4_5 $$ | $$\pi/2$$ | 0 | 0 | $$ \theta_5$$ |
$$ T^5_6 $$ | $$-\pi/2$$ | 0 | 0 | $$ \theta_6 $$ |
$$ T^6_{gripper} $$ | 0 | 0 | 0.453 | 0 |

Note: The end effector is located where the gripper fingers are located

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.


The tranformation martices about each joint follow the following equation:

$$^{i-1}_i T =
 \begin{pmatrix}
  c\theta_i & -s\theta_i & 0 & a_{i-1} \\
  s\theta_i c\alpha_{i-1} & c\theta_i c\alpha_{i-1} & -s\alpha_{i-1} & -s\alpha_{i-1} d_i \\
   s\theta_i s\alpha_{i-1} & c\theta_i s\alpha_{i-1} & c\alpha_{i-1} & c\alpha_{i-1} d_i \\
  0 & 0 & 0 & 1
 \end{pmatrix}$$

We will use the DH parameters to calculate these matrices

$$^0_1 T =
 \begin{pmatrix}
  c\theta_1 & -s\theta_1 & 0 & 0 \\
  s\theta_1 & c\theta_1  & 0 & 0 \\
  0 & 0 & 1 & 0.75 \\
  0 & 0 & 0 & 1
 \end{pmatrix}$$

$$^1_2 T =
 \begin{pmatrix}
  s\theta_2 & c\theta_2 & 0 & 0.35 \\
  0 & 0 & 1 & 0 \\
  c\theta_2 & -s\theta_2  & 0 & 0 \\
  0 & 0 & 0 & 1
 \end{pmatrix}$$

$$^2_3 T =
 \begin{pmatrix}
  c\theta_3 & -s\theta_3 & 0 & 1.25 \\
  s\theta_3 & c\theta_3  & 0 & 0 \\
  0 & 0 & 1 & 0 \\
  0 & 0 & 0 & 1
 \end{pmatrix}$$

$$^3_4 T =
 \begin{pmatrix}
  c\theta_4 & -s\theta_4 & 0 & -0.054 \\
  0 & 0 & 1 & 1.5 \\
  -s\theta_4 & -c\theta_4  & 0 & 0 \\
  0 & 0 & 0 & 1
 \end{pmatrix}$$

$$^4_5 T =
 \begin{pmatrix}
  c\theta_5 & -s\theta_5 & 0 & 0 \\
  0 & 0 & -1 & 0 \\
  s\theta_5 & c\theta_5  & 0 & 0 \\
  0 & 0 & 0 & 1
 \end{pmatrix}$$
 
$$^5_6 T =
 \begin{pmatrix}
  c\theta_6 & -s\theta_6 & 0 & 0 \\
  0 & 0 & 1 & 0 \\
  -s\theta_6 & -c\theta_6  & 0 & 0 \\
  0 & 0 & 0 & 1
 \end{pmatrix}$$
 
$$^6_{gripper} T =
 \begin{pmatrix}
  1 & 0& 0 & 0 \\
  0 & 1 & 0 & 0 \\
  0 & 0  & 1 & 0.453 \\
  0 & 0 & 0 & 1
 \end{pmatrix}$$


Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | L1 | q1
1->2 | - pi/2 | L2 | 0 | -pi/2 + q2
2->3 | 0 | 0 | 0 | 0
3->4 |  0 | 0 | 0 | 0
4->5 | 0 | 0 | 0 | 0
5->6 | 0 | 0 | 0 | 0
6->EE | 0 | 0 | 0 | 0


#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

In this exersice we are given the final postion of the end effector (EE). This is both the x,y,z positions of the EE and the orientations: roll, pitch and yaw. We then need to return joint angles needed in order to achieve this pose. We will denote the x,y,z corrdinate vector as $$V_{EE}$$

It is possible to decouple this complex IK problem into two simpler problems: an "Inverse Position Kinematics" and "Inverse Orientation Kinematics" problems. We will use the "Inverse Position Kinematics" part to findd the angles of joints 1,2 and 3. We will then use the "Inverse Orientation Kinematics" to find the angles of joints 4,5 and 6. In order to do this we first need find the x,y and z coordinates of the Wrist Center (WC). The WC is going to be located in joint 5. We will use the desired position and orientation of the end effector together with the configuration of the kuka arm in order to calculate this. 

From the dimensions of the kuka arm, we know that the WC is located -0.453 m in the z direction in the EE (end effector) reference frame. So what we need is to find the z unit vector for the EE reference frame and represent it in the world reference frame. The unit vector in the z direction of the EE reference frame represented in the world reference frame is the last column of the rotation matrix between the world reference frame and the EE reference frame. To find this rotation matrix we will use desired roll, pitch and yaw of the final EE position. The equation is:

$$R_{EE} = R_{Yaw} * R_{Pitch} * R_{Roll} * R_{Error}$$ 

The right most roatation matrix is the error rotation matrix. It is used to fix the difference between the rotation between the world reference frame and the EE reference frame when the roll, pitch, yaw and all the joint angles are zero. The error rotation in our case is the following:

 $$R_{Error} = \begin{pmatrix}
  0 & 0& -1 \\
  0 & 1 & 0  \\
  1 & 0  & 0 
 \end{pmatrix} $$

As we stated before the last column of the EE rotation matrix is the z unit vector of the EE reference frame expressed in the world reference frame. We shall denote this vector as $$Z_{EE}$$

So the equation for finding the WC is

$$WC = V_{EE} - d_7 * Z_{EE}$$

####Inverse Position Kinematics Analysis

In order to calculate the first joint angle we will use the following equation:

$$ \theta_1 = cos^{-1}(WC_Y/WC_X)$$

Where:

$$WC = \begin{pmatrix}
  WC_X\\
  WC_Y  \\
  WC_Z 
 \end{pmatrix} $$

In order to derive the second and third joint angles we shall use the following image

![alt text][image2]

$$angle\_a = cos_{-1}(\frac{side\_b^2 + side\_c^2 - side\_a^2}{2*side\_b*side\_c})$$
$$angle\_b = cos_{-1}(\frac{side\_a^2 + side\_c^2 - side\_b^2}{2*side\_a*side\_c})$$
$$angle\_c = cos_{-1}(\frac{side\_a^2 + side\_b^2 - side\_c^2}{2*side\_a*side\_b})$$

$$angle\_a2 = tan^{-1}(\frac{WC_Z - d_1}{\sqrt{WC_X ^2+WC_Y^2}-a_1})$$

$$angle\_b2 = tan^{-1}(-a_3/d_4)$$

$$\theta_2 = \pi/2 - (angle\_a + angle\_a2)$$
$$\theta_3 = \pi/2 - (angle\_b + angle\_b2)$$

####Inverse Orientation Kinematics Analysis

In order to find joint angles 3, 4 and 5 we need to find the rotation matrix
$$^3_6R$$

In order to do this we will use the following equations:

$$^0_{EE}R = ^0_3R * ^3_{EE}R = R_{EE}$$ 

Where R_EE is the desired EE position which we are given and have already derived its equation.

We can continue to develope these equations:

$$ ^3_{EE}R  = ^3_0R * R_{EE} = (^0_3R)^{-1} * R_{EE} = (^0_3R)^T * R_{EE} $$

We should note that:

$$^3_{EE}R = ^3_6R * ^3_6{EE} = ^3_6R * I = ^3_6R $$

In order to find the rotation matrix from joint 0 to joint 3 we will use the transformation matrix taking the first three rows and culons of that matrix. 

$$ ^0_3T = ^0_1T * ^1_2T * ^2_3T $$
 
We will insert the joint angles 1, 2 and 3 in order to calculate this rotation matrix.

In order to find the last three theta angles we need to analyis the following matrix


$$^3_6 T =
 \begin{pmatrix}
  c\theta_4c\theta_5c\theta_6-s\theta_4s\theta_6 & -c\theta_4c\theta_5s\theta_6-s\theta_4c\theta_6 & -c\theta_4s\theta_5 \\
  s\theta_5c\theta_6 & -s\theta_5s\theta_6 & c\theta_5 \\
   -s\theta_4c\theta_5c\theta_6-c\theta_4s\theta_6 & s\theta_4c\theta_5s\theta_6-c\theta_4s\theta_6 & s\theta_4s\theta_5
 \end{pmatrix}$$
 

 We can use this to calculate the thetas: 
 
 $$ \theta_4 = tan^{-1}(-^3_6R_{33}/^3_6R_{13})$$
 
 $$\theta_5 = tan^{-1}(\frac{\sqrt{^3_6R_{13}^2+^3_6R_{33}^2}}{^3_6R_{23}})$$
 
 $$\theta_6 = tan^{-1}(-^3_6R_{22}/^3_6R_{21})$$

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


When filling in the 'IK_server.py' I started off defining all the symbols that I will be using through the program. The next step was to calculate the transform matrix between the different joints. In order to make this simpler I created a function that given alpha, a, d, and q returns the transform matrix. The next step was to calculate all the transform matrixes. 

The next step was to do the inverse position kinematics and then the inverse orientation kinematics. In order to do that wrist center (WC). To find this I calculated rotation matrix from the given roll, pitch and yaw. I then used the equations we developed to find the WC and to calculate the rotation matrix R0_EE which is the desired rotation between the world coordinate system and the EE coordinate system.

Once I find the WC I calculated thetas 1-3 using the equations.

In order to calculate thetas 4-6 I needed matrix R3_6. To calculate R3_6 I calculated R0_3 and R0_EE. We have alread calculated R0_EE. In order to calculate R0_3 I used T0_1, T1_2, T2_3 that I calculated in the begining. I then substituted the thetas 1-3 with the thetas we calculated. 

####Improvements

This code works but is far from being practical. There are desired end effector positions that have several solutions. This is espcecially true in cases where there is gimble lock when joint 5 angle is zero. In this case joint 4 and 5 have infinate solutions. The result of this ambiguity is that sometimes the robotic hand when moving from one position to another chooses to configurations that are drastically different from each other. This results non smooth movement where the robot completely changes all its joint angles. 

If I were to presue this project further I could try to resolve this issue. One way to do this is in each of the inverse kinematics calculation step, find several plausible solutions. Then next step is from all possible solutions pick the solution that minimizes the change in the joint angles from the previous step.
