function robot = buildRobotForCodegen()
%BUILDROBOTFORCODEGEN Construct the rigidBodyTree for code generation.
%#codegen

robot = rigidBodyTree('DataFormat', 'column', 'MaxNumBodies', 12);
robot.Gravity = [0 0 -9.81];

% Planar base joints (X, Y, Yaw)
addBodyWithJoint(robot, 'base_link_x', 'base', 'joint_x', 'prismatic', ...
    [0 0 0], [0 0 0], [1 0 0], [-50 50]);
addBodyWithJoint(robot, 'base_link_y', 'base_link_x', 'joint_y', 'prismatic', ...
    [0 0 0], [0 0 0], [0 1 0], [-50 50]);
addBodyWithJoint(robot, 'abstract_chassis_link', 'base_link_y', 'joint_theta', 'revolute', ...
    [0 0 0], [0 0 0], [0 0 1], [-pi pi]);

% Fixed mount for the arm
addBodyWithJoint(robot, 'left_arm_base_link', 'abstract_chassis_link', 'arm_mount_joint', 'fixed', ...
    [0.15995 0 0.9465], [0 0 -pi/2], [0 0 1], []);

% Arm joints 1-6
addBodyWithJoint(robot, 'left_arm_link1', 'left_arm_base_link', 'left_arm_joint1', 'revolute', ...
    [-0.0011149 0.0446 0], [0 0 0], [0 1 0], [-2.8798 2.8798]);
addBodyWithJoint(robot, 'left_arm_link2', 'left_arm_link1', 'left_arm_joint2', 'revolute', ...
    [0 0.1061 0], [0 0 0], [0 0 -1], [0 3.2289]);
addBodyWithJoint(robot, 'left_arm_link3', 'left_arm_link2', 'left_arm_joint3', 'revolute', ...
    [-0.34928 0.019998 0], [0 0 0], [0 0 -1], [-3.3161 0]);
addBodyWithJoint(robot, 'left_arm_link4', 'left_arm_link3', 'left_arm_joint4', 'revolute', ...
    [0.02735 0.069767 0], [0 0 0], [1 0 0], [-2.8798 2.8798]);
addBodyWithJoint(robot, 'left_arm_link5', 'left_arm_link4', 'left_arm_joint5', 'revolute', ...
    [0.2463 -0.00049894 0], [0 0 0], [0 1 0], [-1.6581 1.6581]);
addBodyWithJoint(robot, 'left_arm_link6', 'left_arm_link5', 'left_arm_joint6', 'revolute', ...
    [0.058249 0.00050025 0], [0 0 0], [1 0 0], [-2.8798 2.8798]);

% Fixed gripper link
addBodyWithJoint(robot, 'left_gripper_link', 'left_arm_link6', 'left_gripper_joint', 'fixed', ...
    [0.1039 0 0], [0 0 0], [0 0 1], []);

end

function addBodyWithJoint(tree, bodyName, parentName, jointName, jointType, xyz, rpy, axis, limits)
body = rigidBody(bodyName);
joint = rigidBodyJoint(jointName, jointType);
tform = eul2tform(rpy, 'XYZ');
tform(1:3,4) = xyz;
setFixedTransform(joint, tform);
if ~strcmp(jointType, 'fixed')
    joint.JointAxis = axis;
    if ~isempty(limits)
        joint.PositionLimits = limits;
    end
end
body.Joint = joint;
addBody(tree, body, parentName);
end

