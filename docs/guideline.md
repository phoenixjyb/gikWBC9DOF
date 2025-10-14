Hierarchical Task Prioritization
The essence of the "cameraman" control design is establishing a clear hierarchy of these tasks. The primary task is paramount. If a conflict arises—for example, if following the trajectory perfectly requires colliding with an obstacle—the system must have a defined way to arbitrate. In a soft-prioritization scheme, the controller would find a compromise that results in a small trajectory tracking error in order to maintain a safe distance from the obstacle. This compromise is governed by the relative "weights" assigned to each task in the optimization problem.

This paradigm of continuous, velocity-level coordination is key. The robot is not planning a path, stopping, re-planning, and then moving again. It is continuously adjusting its joint velocities in real-time to satisfy all objectives according to their priority. This points toward control architectures that operate at a high frequency on velocity commands. In the context of ROS 2, this strongly favors moveit_servo , which is designed for this exact purpose. In MATLAB, it implies that the GIK solver must be called within a high-frequency control loop, solving for small configuration changes at each time step to approximate this continuous, fluid motion.   

Part II: Design and Implementation with MATLAB Robotics System Toolbox
The MATLAB Robotics System Toolbox provides a powerful and flexible environment for modeling, simulating, and designing control algorithms for complex robotic systems. Its generalizedInverseKinematics (GIK) solver is particularly well-suited for implementing the multi-objective, constraint-driven control required for the 9-DOF mobile manipulator.

Constructing the Digital Twin: The rigidBodyTree Model
The rigidBodyTree object is the cornerstone of kinematic and dynamic analysis in the toolbox. It represents the robot as a tree structure of rigid bodies connected by joints. A crucial first step is to accurately model the entire 9-DOF mobile manipulator.   

Modeling the P-P-R Planar Base
The rigidBodyTree model does not have a native "planar" joint type that can be directly used by the IK solvers in the same way as in ROS. Therefore, the 3-DOF Prismatic-Prismatic-Revolute (P-P-R) base must be constructed as a serial chain of three individual single-DOF joints.   

Initialize the Robot Model: Create an empty rigidBodyTree object. It is good practice to specify the data format, which is required for dynamics functions and code generation.   

Matlab

robot = rigidBodyTree('DataFormat','column');
Create the X-Axis Prismatic Joint: Define a rigid body for the first stage of motion and a prismatic joint. The JointAxis property defines the direction of motion. This body is attached to the world frame, referred to as robot.BaseName.

Matlab

body_x = rigidBody('base_x_link');
joint_x = rigidBodyJoint('base_joint_x','prismatic');
joint_x.JointAxis = [1 0 0];
body_x.Joint = joint_x;
addBody(robot, body_x, robot.BaseName);
Create the Y-Axis Prismatic Joint: Define the next body and attach it to the previous one.

Matlab

body_y = rigidBody('base_y_link');
joint_y = rigidBodyJoint('base_joint_y','prismatic');
joint_y.JointAxis = [0 1 0];
body_y.Joint = joint_y;
addBody(robot, body_y, 'base_x_link');
Create the Z-Axis Revolute Joint: Define the final body of the base, which provides the rotation.

Matlab

body_theta = rigidBody('base_theta_link');
joint_theta = rigidBodyJoint('base_joint_theta','revolute');
joint_theta.JointAxis = [0 0 1];
body_theta.Joint = joint_theta;
addBody(robot, body_theta, 'base_y_link');
This explicit serial chain representation, while more verbose than a single virtual joint, provides direct, individual access to each of the base's DOFs within the GIK framework, which is highly advantageous for applying fine-grained constraints.

Integrating the Manipulator Arm
Assuming the 6-DOF arm is described in a URDF file, it can be imported as a separate rigidBodyTree and then grafted onto the base model.

Import the Arm: Use importrobot to create a model of the arm.

Matlab

arm = importrobot('path/to/your/arm.urdf');
Attach the Arm Subtree: Use addSubtree to attach the base of the arm model to the final rotating link of the planar base model.

Matlab

addSubtree(robot, 'base_theta_link', arm);
Verification and Inspection
After construction, it is essential to verify the complete model.

Inspect the Structure: Use showdetails(robot) to print a summary of the kinematic chain, confirming that all 9 non-fixed joints are present with the correct parent-child relationships.   

Visualize the Model: Use show(robot, homeConfiguration(robot)) to display the robot in its home configuration. Further test with randomConfiguration(robot) to ensure the joint limits are correctly defined and the model behaves as expected.   

Mastering Multi-Objective Control with Generalized Inverse Kinematics (GIK)
For a redundant robot with multiple, potentially conflicting objectives, the standard inverseKinematics solver is insufficient. It is designed to solve for a single end-effector pose and does not provide a mechanism for incorporating additional constraints like obstacle avoidance or joint-space preferences.   

The generalizedInverseKinematics (GIK) object is the correct and necessary tool for this problem. It is explicitly designed as a multi-constraint solver, framing the IK problem as a nonlinear optimization task. The solver, typically using an algorithm like BFGS Gradient Projection, attempts to find a joint configuration    

q that minimizes a cost function derived from the "violation" of all active constraints.   

Setting Up the GIK Solver
Configuration of the GIK solver is a two-step process:

Instantiation: Create the solver object and associate it with the robot model.

Matlab

gik = generalizedInverseKinematics('RigidBodyTree', robot);
Defining Constraint Inputs: The solver must be told what types of constraint objects to expect, and in what order. This is done by setting the ConstraintInputs property. The order in this cell array must exactly match the order in which the constraint objects are passed to the solver during each call.

Matlab

% Example for a problem with pose, joint bounds, and distance constraints
gik.ConstraintInputs = {'pose', 'joint', 'distance'};
Tuning Solver Parameters
The SolverParameters property allows for tuning the behavior of the underlying optimization algorithm. For trajectory tracking, where smoothness and predictability are key, the following parameters are particularly important:   

MaxIterations: The maximum number of iterations the solver will perform. A higher number may find a better solution but will take longer. Default is 1500.

AllowRandomRestarts: When enabled, the solver will restart from a random configuration if it gets stuck in a local minimum. For smooth trajectory tracking, it is crucial to disable this to prevent sudden jumps in the solution: gik.SolverParameters.AllowRandomRestart = false;.   

SolutionTolerance: The tolerance on the satisfaction of the constraints. A smaller value leads to a more accurate solution.

A Constraint-Driven Approach to Control Modes
The three required operational modes—freezing the chassis, freezing the arm, and coordinated whole-body motion—can be implemented elegantly and robustly by dynamically configuring a constraintJointBounds object. This constraint object allows the specification of upper and lower position bounds for each joint in the robot's configuration vector.   

The key is to combine the Bounds property to restrict motion with the Weights property to specify the importance of enforcing those bounds. A weight of 1 indicates a high-priority constraint, while a weight of 0 effectively disables it for that joint.   

Mode 1: Freeze Chassis (Arm-Only Control)
To lock the mobile base, its joints are constrained to their current positions.

Create the Constraint: jointConst = constraintJointBounds(robot);

Get Current Configuration: q_current = robot.homeConfiguration; (or the actual current configuration).

Set Bounds: For the base joints (indices 1, 2, 3 in the configuration vector), set the lower and upper bounds to be equal to the current joint position.

Matlab

jointConst.Bounds(1,:) = [q_current(1).JointPosition, q_current(1).JointPosition]; % For joint_x
jointConst.Bounds(2,:) = [q_current(2).JointPosition, q_current(2).JointPosition]; % For joint_y
jointConst.Bounds(3,:) = [q_current(3).JointPosition, q_current(3).JointPosition]; % For joint_theta
Set Weights: Assign a high weight to the base joints to enforce the lock, and zero weight to the arm joints to allow them to move freely (within their physical limits, which are handled by a separate internal mechanism).

Matlab

num_joints = numel(q_current);
jointConst.Weights = [1, 1, 1, zeros(1, num_joints - 3)];
Mode 2: Freeze Arm (Base-Only Control)
This is the inverse of the previous mode. The six arm joints are locked, while the base is free to move.

Set Bounds: Lock the arm joints (indices 4 through 9) to their current positions.

Set Weights: jointConst.Weights = [0, 0, 0, ones(1, num_joints - 3)];

Mode 3: Coordinated Whole-Body Control
In this mode, all joints are free to move within their physical limits. The constraintJointBounds object is still useful for enforcing these physical limits with a specific weight.

Set Bounds: The bounds are left at their default values, which are inherited from the PositionLimits property of each joint in the rigidBodyTree model.   

Set Weights: All weights are set to 1 to ensure all physical joint limits are respected.

Matlab

jointConst.Weights = ones(1, num_joints);
Primary Task Constraint
Regardless of the mode, the primary goal of reaching the end-effector target is defined using constraintPoseTarget.   

Matlab

poseTgt = constraintPoseTarget('end_effector_link_name');
poseTgt.TargetTransform = desired_pose_matrix; % A 4x4 homogeneous transform
Calling the Solver in a Loop
The solver is then called with an initial guess (typically the solution from the previous timestep) and the set of active constraint objects.

Matlab

% Inside a control loop...
% Update desired_pose_matrix, q_current, and jointConst based on mode
[q_sol, solutionInfo] = gik(q_current, poseTgt, jointConst);
The following table provides a clear, actionable guide for configuring the constraintJointBounds object to achieve each of the three user-specified control modes.

Joint Name	Config. Vector Index	Control Mode	Bounds Setting (jointConst.Bounds(i,:))	Weight Setting (jointConst.Weights(i))	Rationale
base_joint_x	1	Freeze Chassis	[q_current(1), q_current(1)]	1	High weight and tight bounds to lock the joint.
base_joint_y	2	Freeze Chassis	[q_current(2), q_current(2)]	1	High weight and tight bounds to lock the joint.
base_joint_theta	3	Freeze Chassis	[q_current(3), q_current(3)]	1	High weight and tight bounds to lock the joint.
arm_joint_...	4-9	Freeze Chassis	Default (Physical Limits)	0	Zero weight allows free motion for the arm.
base_joint_...	1-3	Freeze Arm	Default (Physical Limits)	0	Zero weight allows free motion for the base.
arm_joint_1	4	Freeze Arm	[q_current(4), q_current(4)]	1	High weight and tight bounds to lock the joint.
arm_joint_2	5	Freeze Arm	[q_current(5), q_current(5)]	1	High weight and tight bounds to lock the joint.
arm_joint_...	6-9	Freeze Arm	[q_current(i), q_current(i)]	1	High weight and tight bounds to lock the joints.
base_joint_...	1-3	Coordinated	Default (Physical Limits)	1	Enforces physical limits for all base joints.
arm_joint_...	4-9	Coordinated	Default (Physical Limits)	1	Enforces physical limits for all arm joints.

Export to Sheets
Advanced Behaviors: Obstacle Avoidance and Task Prioritization
The true power of the GIK framework lies in its ability to combine the primary task and operational mode constraints with additional constraints for more complex behaviors.

Reactive Obstacle Avoidance
The constraintDistanceBounds object is the primary tool for implementing reactive collision avoidance. It constrains the distance between the origin of one body (on the robot) and the origin of a reference body (representing an obstacle).   

For each critical link on the robot and each detected obstacle, a constraint is created.

A frame representing the obstacle's center must be available.

The lower bound of the constraint is set to a desired safety margin, while the upper bound is infinite.

Matlab

% Assume 'obstacle_frame' is a body representing the obstacle
distConst = constraintDistanceBounds('robot_critical_link');
distConst.ReferenceBody = 'obstacle_frame';
distConst.Bounds = [0.2, Inf]; % Maintain a 20 cm safety margin
"Cameraman" Gaze Maintenance
To ensure the end-effector (camera) remains pointed at a specific point in space, regardless of its position, the constraintAiming object is used. This constraint forces the z-axis of the specified end-effector frame to align with the vector pointing from the end-effector's origin to a target point.   

Matlab

aimConst = constraintAiming('camera_link'); % Assuming 'camera_link' is the end-effector
aimConst.TargetPoint = [target_x, target_y, target_z];
Task Prioritization via the Weights Property
When multiple constraints are active, they may conflict. For example, perfectly tracking a trajectory might require moving through an obstacle. The GIK solver resolves these conflicts based on the Weights assigned to each constraint. The Weights property effectively defines the cost of violating a given constraint, allowing for a soft, prioritized task hierarchy.   

Primary Task: The constraintPoseTarget for trajectory tracking should have the highest weights to signify its importance.

Matlab

poseTgt.Weights = [1 1]; % High weight for both orientation and position
Secondary Tasks: Obstacle avoidance constraints should have a lower, but still significant, weight.

Matlab

distConst.Weights = 0.5;
Tertiary Tasks: Posture optimization or aiming constraints might have the lowest weights.

Matlab

aimConst.Weights = 0.1;
With this weighting scheme, the solver will prioritize finding a solution that satisfies the poseTgt. If necessary, it will allow a small violation of the distConst (i.e., dipping slightly into the safety margin) if that is the only way to satisfy the high-weight pose constraint. This mechanism is how the "cameraman" can slightly alter its path to get around an obstacle while keeping the camera shot as stable as possible. The MATLAB GIK framework, through this combination of diverse constraint objects and tunable weights, provides a user-friendly interface to a complex constrained optimization problem, mirroring the principles of advanced hierarchical whole-body control without requiring manual formulation of quadratic programs.

Pathway to Real-Time Deployment
To move from simulation to a real-time application, the GIK-based control logic must be executed in a high-frequency loop and, for performance, compiled into C++.

Trajectory Following Control Loop
The core of the application is a loop that iterates through the desired trajectory waypoints.

Matlab

% Pre-computation
gik =...; % Initialize GIK solver
poseTgt =...; % Initialize constraint objects
jointConst =...;
distConst =...;
q_sol = homeConfiguration(robot); % Initial configuration

% Control Loop
for i = 1:num_waypoints
    % 1. Update target pose
    poseTgt.TargetTransform = trajectory_poses(:,:,i);

    % 2. Update obstacle information (from sensors)
    % distConst.ReferenceBody =...;

    % 3. Update joint bounds for current mode
    % jointConst.Bounds =...; jointConst.Weights =...;

    % 4. Solve for the next configuration, using previous as initial guess
    [q_sol, solutionInfo] = gik(q_sol, poseTgt, jointConst, distConst);

    % 5. Send q_sol to the robot controller
    send_joint_commands(q_sol);
    
    % Wait for next control cycle
    waitfor(rateControlObj);
end
Using the previous solution as the initial guess for the next iteration is critical for ensuring smoothness and improving solver convergence speed.   

Code Generation for High Performance
To achieve the performance required for a real-time system, the MATLAB control loop can be converted to C++ using MATLAB Coder.

Create a Code-Generation-Compatible Function: The GIK solver logic must be encapsulated in a MATLAB function. The solver object itself should be declared as persistent so that it is initialized only once.   

Matlab

function q_next = solve_gik_step(q_current, target_pose,...)
    %#codegen
    persistent gik poseTgt jointConst;
    if isempty(gik)
        robot =...; % Load or define robot
        gik = generalizedInverseKinematics('RigidBodyTree', robot,...);
        poseTgt = constraintPoseTarget(...);
        jointConst = constraintJointBounds(robot);
    end

    % Update constraint parameters
    poseTgt.TargetTransform = target_pose;
    %... update other constraints

    % Solve
    q_next = gik(q_current, poseTgt, jointConst);
end
Generate C++ Code: Use the codegen command to generate a MEX function for use in MATLAB or standalone C/C++ source code for integration into an external project.

Matlab

% Generate a MEX file for faster execution within MATLAB
codegen solve_gik_step -args {initial_q, initial_pose,...}
This generated C++ function can then be called from a low-latency real-time framework, providing the performance necessary for smooth and reactive whole-body control