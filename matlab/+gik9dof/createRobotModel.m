function robot = createRobotModel(options)
%CREATEROBOTMODEL Assemble the 9-DOF mobile manipulator as a rigidBodyTree.
%   robot = GIK9DOF.CREATEROBOTMODEL() loads the robot definition from the
%   project URDF, normalises the planar base joint axes, and prepares the
%   model for use with generalized inverse kinematics.
%
%   robot = GIK9DOF.CREATEROBOTMODEL(options) allows configuration via name
%   value pairs:
%       UrdfPath   - Path to the URDF file (string, default resolves to the
%                    repository copy of mobile_manipulator_PPR_base_corrected.urdf).
%       MeshPath   - Directory that stores STL meshes used for visuals and
%                    collision geometry (string, default resolves to
%                    <repo>/meshes).
%       DataFormat - Desired data format for the rigidBodyTree. Either
%                    'column', 'row', or 'struct'. Default is 'column',
%                    which matches the MATLAB Robotics System Toolbox GIK
%                    requirements.
%       Gravity    - 1-by-3 vector specifying the gravity vector, default
%                    [0 0 -9.81].
%       AdjustPlanarBase - Logical flag (default true) that forces the
%                    prismatic joints of the planar base to align with the
%                    world X and Y axes and the revolute joint with Z.
%       Validate   - Logical flag (default false). When true an assertion is
%                    issued if the imported model does not expose the
%                    expected joint set.
%
%   The function keeps the manipulator subtree exactly as defined in the
%   URDF while reshaping the base joints so the controller can reason about
%   world-frame Cartesian motions cleanly.
%
%   The helper relies on the MATLAB Robotics System Toolbox.
%
%   Example:
%       robot = gik9dof.createRobotModel();
%       showdetails(robot)
%
%   See also generalizedInverseKinematics, importrobot.

arguments
    options.UrdfPath (1,1) string = "mobile_manipulator_PPR_base_corrected.urdf"
    options.MeshPath (1,1) string = "meshes"
    options.DataFormat (1,1) string = "column"
    options.Gravity (1,3) double = [0 0 -9.81]
    options.AdjustPlanarBase (1,1) logical = true
    options.Validate (1,1) logical = false
end

% Resolve URDF and mesh paths relative to the repository when necessary.
urdfPath = gik9dof.internal.resolvePath(options.UrdfPath);
meshPath = gik9dof.internal.resolvePath(options.MeshPath, true);

if ~isfile(urdfPath)
    error("gik9dof:createRobotModel:MissingUrdf", ...
        "URDF file not found: %s", urdfPath);
end

if ~isfolder(meshPath)
    error("gik9dof:createRobotModel:MissingMeshes", ...
        "Mesh directory not found: %s", meshPath);
end

% Import the full mobile manipulator as described in the URDF. The
% resulting tree already includes the planar base encoded via individual
% prismatic and revolute joints.
robot = importrobot(urdfPath, ...
    "DataFormat", options.DataFormat, ...
    "MeshPath", meshPath);
robot.Gravity = options.Gravity;

if options.AdjustPlanarBase
    % The URDF uses individual prismatic joints to emulate a planar base.
    % Make sure their joint axes align with the world frame to keep the
    % solver numerics intuitive.
    setJointAxisIfPresent(robot, "base_link_x", [1 0 0]);
    setJointAxisIfPresent(robot, "base_link_y", [0 1 0]);
    setJointAxisIfPresent(robot, "abstract_chassis_link", [0 0 1]);
end

if options.Validate
    expected = ["joint_x"; "joint_y"; "joint_theta"; ...
                "left_arm_joint1"; "left_arm_joint2"; "left_arm_joint3"; ...
                "left_arm_joint4"; "left_arm_joint5"; "left_arm_joint6"];
    jointNames = strings(1, numel(robot.Bodies));
    for i = 1:numel(robot.Bodies)
        jointNames(i) = string(robot.Bodies{i}.Joint.Name);
    end
    names = jointNames;
    if ~all(ismember(expected, names))
        error("gik9dof:createRobotModel:UnexpectedStructure", ...
            "Imported robot is missing one or more expected joints.\nExpected: %s\nGot: %s", ...
            strjoin(expected, ", "), strjoin(names, ", "));
    end
end
end

function setJointAxisIfPresent(robot, bodyName, axisVector)
%SETJOINTAXISIFPRESENT Helper that guards against missing bodies.
try
    body = robot.getBody(bodyName);
catch
    warning("gik9dof:createRobotModel:MissingBody", ...
        "Body '%s' not found while adjusting joint axis.", bodyName);
    return
end
joint = body.Joint;
if strcmpi(joint.Type, "fixed")
    % Skip fixed joints; nothing to align.
    return
end
joint.JointAxis = axisVector;
end
