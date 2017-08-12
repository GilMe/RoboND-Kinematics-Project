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
[image2]: ./misc_images/misc2.png
[image3]: ./misc_images/misc3.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

We will start the forward kinematic analysis by analyzing the urdf file in order to get the robot dimension and get the joint locations that will help us derive the DH parameters. The URDF reference frame is different than the DH reference frame. The axis are the joint axis. Here is a schematic that shows the joint axis locations and orientations

![alt text][image1]

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
$$ T^6_{gripper} $$ | 0 | 0 | 0.303 | 0 |

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
  0 & 0  & 1 & 0.303 \\
  0 & 0 & 0 & 1
 \end{pmatrix}$$
 
Here's | A | Snappy | Table
--- | --- | --- | ---
1 | `highlight` | **bold** | 7.41
2 | a | b | c
3 | *italic* | text | 403
4 | 2 | 3 | abcd

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

And here's another image! 

![alt text][image2]

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  


And just for fun, another example image:
![alt text][image3]


