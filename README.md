[theta1]: ./images/theta1.jpg
[theta23]: ./images/theta23.jpg
[theta456]: ./images/theta456.jpg
## Project: Kinematics Pick & Place
### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.
I went off what I saw in the third video of forward kinematics for the kr210 while looking at the kr210.urdf.xacro file to ensure the parameters actually made sense.  Below is an image breaking down how each value from the urdf fits into the the derived DH parameters.

**DH Parameter Table**

T | alpha | d (offset) | a
--- | --- | --- | ---
T0_1 | 0 | 0.75 | 0
T1_2 | -pi/2 | 0 | .35
T2_3 | 0 | 0 | 1.25
T3_4 | -pi/2 | 1.5 | -0.054
T4_5 | pi/2 | 0 | 0
T5_6 | -pi/2 | 0 | 0
T6_G | 0 | .303 | 0

**Account for -pi/2 offset on q2 as noted in lesson and rotation correction to put gripper back in world frame:**
q2 = q2 - pi/2
Rotation along Z axis by pi
Rotation along Y axis by -pi/2

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

I used the quarternion coordinates for this calculation (hadn't looked at the IK_server code and seen the calculation to RPY): 
    
    R0_G = [
        [1 - 2*qy**2 - 2*qz**2, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw,      px]
        [2*qx*qy + 2*qz*qw, 1 - 2*qx**2 - 2*qz**2, 2*qy*qz - 2*qx*qw,      py]
        [2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1 - 2*qx**2 - 2*qy**2,      pz]
        [0                ,0                ,0                    ,         1]
        ]

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.
To find the wrist center I get the R0_G rotation matrix by using the quarterion coordinates which is then post multiplied by the rotation correction matrix's transpose.  The vector orthonormal to the local z-axis of the gripper is then used to derive the WC as instructed in the inverse kinematic lesson (as they are only offset along that axis).  The rotation reversal isn't necessary (could just keep it unrotated and use local x-axis directional vector to get same result) but I preferred to stay consistent with the lesson.
After decoupling the inverse position and orientation kinematics into j1,2,3 and j4,5,6 respectively j1 is the only joint that rotates along the Z (world) axis among the "positional" joints.  Conversely, this means that the angle rotated along the Z-axis is theta 1 which can be determined with atan2(y,x).

![alt text][theta1]

Joint 2 and Joint 3 work together to determine the Z (world) position of the WC.  Using the T1 coordinate system shifted .35 (a1) on the positive X axis to remove any offset, we're left with a triangle consisting of the lengths of a2, u=sqrt(a3^2 + d4^2), and the hypotenuse of the shifted local coordinate system.  

![alt text][theta23]

For j4,5,6 as described in the lesson:
R3_0 * R0_6 = R3_6
R3_0 = Inverse(R0_3) which means R3_0 is filled with numerical values because we have j1,2,3.
R0_6 = Rrpy can be derived from using the quarterion coordinates to get the rotational matrix
This means we have the rotational matrix R3_6 completely filled out with values.  To find the equations to derive J4,5,6 T3-6 were post multiplied together to get T3_6 and the rotational portion was taken (R3_6).  The equations at each index of the matrix R3_6 (after premultiplying it by a a rotational matrix that goes from the world frame (X, Y, Z) to R4s reference frame (Z,-Y,X), to start it at the same reference of rotation) correspond to the numerical values we've derived from R3_0 * Rrpy

![alt text][theta456]

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 

The code implemented finds both possibilities for theta 1 and loops through them.  It then loops through the two possibilities of theta2 and 3 for the current theta 1. Another loop within this is again done for theta4/5/6.  If theta 4 and 5 don't match up using the initial theta 4 atan calculation I add pi or subtract pi to theta4 to make it complement theta 5s angle.  This is done pretty sloppily which can lead to the ocassional unnecessary turn by theta 4 (going clockwise 359 degrees instead of going 1 degree clockwise for example).  Theta6 isnt adjusted even if the second "verifying" calculation doesn't match up because I assume the gripper grips equally well regardless of if its upside down or not (I may be wrong in this assumption). I also don't do any bounds checking on the joint angle calculations which could be an issue. My results were generally good but fixing these things would make it more reliable. 

