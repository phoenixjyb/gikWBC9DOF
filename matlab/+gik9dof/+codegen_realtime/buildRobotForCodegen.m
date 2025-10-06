function robot = buildRobotForCodegen()
%BUILDROBOTFORCODEGEN Build 9-DOF mobile manipulator procedurally for code generation
%   This function constructs the rigidBodyTree without any file I/O,
%   making it compatible with MATLAB Coder for C++ code generation.
%
%   Robot structure: 3 base DOFs (x, y, theta) + 6 arm DOFs
%   All inertial, visual, and joint parameters are hardcoded from URDF.
%
%#codegen

% Create rigid body tree with column data format (required for GIK)
robot = rigidBodyTree('DataFormat', 'column');
robot.Gravity = [0 0 -9.81];

%% BASE JOINTS (3 DOF planar base: x, y, theta)

% Joint 1: Prismatic X (joint_x)
baseX = rigidBody('base_link_x');
baseX.Mass = 1.0;
baseX.CenterOfMass = [0 0 0];
baseX.Inertia = [1.0 1.0 1.0 0 0 0];
jntX = rigidBodyJoint('joint_x', 'prismatic');
setFixedTransform(jntX, trvec2tform([0 0 0]));
jntX.JointAxis = [1 0 0];
jntX.PositionLimits = [-50.0 50.0];
jntX.HomePosition = 0;
baseX.Joint = jntX;
addBody(robot, baseX, robot.BaseName);

% Joint 2: Prismatic Y (joint_y)
baseY = rigidBody('base_link_y');
baseY.Mass = 1.0;
baseY.CenterOfMass = [0 0 0];
baseY.Inertia = [1.0 1.0 1.0 0 0 0];
jntY = rigidBodyJoint('joint_y', 'prismatic');
setFixedTransform(jntY, trvec2tform([0 0 0]));
jntY.JointAxis = [0 1 0];
jntY.PositionLimits = [-50.0 50.0];
jntY.HomePosition = 0;
baseY.Joint = jntY;
addBody(robot, baseY, 'base_link_x');

% Joint 3: Revolute Theta (joint_theta)
chassis = rigidBody('abstract_chassis_link');
chassis.Mass = 50.96231322;
chassis.CenterOfMass = [0.00703065 -0.00001010 0.21569964];
chassis.Inertia = [3.27070589 3.37047588 2.72731093 0.00010827 -0.18468600 0.00035735];
jntTheta = rigidBodyJoint('joint_theta', 'revolute');
setFixedTransform(jntTheta, trvec2tform([0 0 0]));
jntTheta.JointAxis = [0 0 1];
jntTheta.PositionLimits = [-inf inf];
jntTheta.HomePosition = 0;
chassis.Joint = jntTheta;
addBody(robot, chassis, 'base_link_y');

%% ARM BASE LINK (fixed mount on chassis)
armBase = rigidBody('left_arm_base_link');
armBase.Mass = 1.658;
armBase.CenterOfMass = [-0.0005634 0.038934 3.1874E-06];
armBase.Inertia = [0.0010597 0.0011787 0.0010647 1.9821E-05 -1.6752E-07 -1.9146E-07];
jntArmMount = rigidBodyJoint('arm_mount_joint', 'fixed');
% Origin: rpy="0 0 -1.5708" xyz="0.15995 0 0.9465"
setFixedTransform(jntArmMount, trvec2tform([0.15995 0 0.9465]) * eul2tform([0 0 -1.5708], 'XYZ'));
armBase.Joint = jntArmMount;
addBody(robot, armBase, 'abstract_chassis_link');

%% ARM JOINT 1 (revolute, Y-axis)
armLink1 = rigidBody('left_arm_link1');
armLink1.Mass = 1.164;
armLink1.CenterOfMass = [0.000015 0.105259 -0.001954];
armLink1.Inertia = [0.001125 0.001084 0.001158 0.0 0.0 -0.000023];
jntArm1 = rigidBodyJoint('left_arm_joint1', 'revolute');
setFixedTransform(jntArm1, trvec2tform([-0.0011149 0.0446 0]));
jntArm1.JointAxis = [0 1 0];
jntArm1.PositionLimits = [-2.8798 2.8798];
jntArm1.HomePosition = 0;
armLink1.Joint = jntArm1;
addBody(robot, armLink1, 'left_arm_base_link');

%% ARM JOINT 2 (revolute, -Z-axis)
armLink2 = rigidBody('left_arm_link2');
armLink2.Mass = 1.3;
armLink2.CenterOfMass = [-0.23622 0.016352 -0.00013275];
armLink2.Inertia = [0.00060638 0.0075936 0.0075712 0.00041817 0.00014956 -8.0916E-06];
jntArm2 = rigidBodyJoint('left_arm_joint2', 'revolute');
setFixedTransform(jntArm2, trvec2tform([0 0.1061 0]));
jntArm2.JointAxis = [0 0 -1];
jntArm2.PositionLimits = [0 3.2289];
jntArm2.HomePosition = 0;
armLink2.Joint = jntArm2;
addBody(robot, armLink2, 'left_arm_link1');

%% ARM JOINT 3 (revolute, -Z-axis)
armLink3 = rigidBody('left_arm_link3');
armLink3.Mass = 0.818;
armLink3.CenterOfMass = [0.045114 0.054616 -0.00045593];
armLink3.Inertia = [0.00060107 0.0013959 0.0015027 -0.00022467 -7.1194E-06 -9.7503E-06];
jntArm3 = rigidBodyJoint('left_arm_joint3', 'revolute');
setFixedTransform(jntArm3, trvec2tform([-0.34928 0.019998 0]));
jntArm3.JointAxis = [0 0 -1];
jntArm3.PositionLimits = [-3.3161 0];
jntArm3.HomePosition = 0;
armLink3.Joint = jntArm3;
addBody(robot, armLink3, 'left_arm_link2');

%% ARM JOINT 4 (revolute, X-axis)
armLink4 = rigidBody('left_arm_link4');
armLink4.Mass = 0.698;
armLink4.CenterOfMass = [0.24285 0.0023784 1.279E-06];
armLink4.Inertia = [8.45E-05 0.00010174 9.7044E-05 -8.2627E-07 -2.2607E-09 5.3612E-09];
jntArm4 = rigidBodyJoint('left_arm_joint4', 'revolute');
setFixedTransform(jntArm4, trvec2tform([0.02735 0.069767 0]));
jntArm4.JointAxis = [1 0 0];
jntArm4.PositionLimits = [-2.8798 2.8798];
jntArm4.HomePosition = 0;
armLink4.Joint = jntArm4;
addBody(robot, armLink4, 'left_arm_link3');

%% ARM JOINT 5 (revolute, Y-axis)
armLink5 = rigidBody('left_arm_link5');
armLink5.Mass = 0.417;
armLink5.CenterOfMass = [0.054309 0.0041811 4.0699E-06];
armLink5.Inertia = [8.3999E-05 9.8498E-05 0.00011333 1.6234E-05 7.4127E-08 -1.3811E-08];
jntArm5 = rigidBodyJoint('left_arm_joint5', 'revolute');
setFixedTransform(jntArm5, trvec2tform([0.2463 -0.00049894 0]));
jntArm5.JointAxis = [0 1 0];
jntArm5.PositionLimits = [-1.6581 1.6581];
jntArm5.HomePosition = 0;
armLink5.Joint = jntArm5;
addBody(robot, armLink5, 'left_arm_link4');

%% ARM JOINT 6 (revolute, X-axis)
armLink6 = rigidBody('left_arm_link6');
armLink6.Mass = 0.037;
armLink6.CenterOfMass = [0.028138 1.2134E-07 5.4049E-08];
armLink6.Inertia = [3.5662E-06 2.0238E-06 2.0238E-06 6.6514E-12 2.9628E-12 -4.1666E-12];
jntArm6 = rigidBodyJoint('left_arm_joint6', 'revolute');
setFixedTransform(jntArm6, trvec2tform([0.058249 0.00050025 0]));
jntArm6.JointAxis = [1 0 0];
jntArm6.PositionLimits = [-2.8798 2.8798];
jntArm6.HomePosition = 0;
armLink6.Joint = jntArm6;
addBody(robot, armLink6, 'left_arm_link5');

%% END EFFECTOR (fixed gripper link)
gripper = rigidBody('left_gripper_link');
gripper.Mass = 0.604;
gripper.CenterOfMass = [-0.031107 -1.3893E-07 -1.437E-07];
gripper.Inertia = [0.00017588 9.8637E-05 0.00016512 4.1789E-10 -5.3493E-10 -8.1856E-08];
jntGripper = rigidBodyJoint('left_gripper_joint', 'fixed');
setFixedTransform(jntGripper, trvec2tform([0.1039 0 0]));
gripper.Joint = jntGripper;
addBody(robot, gripper, 'left_arm_link6');

end
