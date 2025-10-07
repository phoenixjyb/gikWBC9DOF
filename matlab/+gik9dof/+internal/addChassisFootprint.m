function info = addChassisFootprint(robot, options)
%ADDCHASSISFOOTPRINT Add footprint reference bodies to the chassis.
%   info = addChassisFootprint(robot) attaches a set of fixed bodies that
%   represent the chassis corners and edge midpoints. These bodies can be
%   referenced in distance constraints to enforce obstacle clearance for
%   the full footprint rather than a single chassis point.
%
%   info = addChassisFootprint(robot, options) allows overriding:
%       options.CornerOffsets   - Nx2 offsets for the corners
%       options.EdgeOffsets     - Mx2 offsets for edge midpoints
%       options.Prefix          - name prefix (default "footprint")
%       options.Height          - Z offset for the reference frames (default 0)
%
%   Returns a struct with fields:
%       Names   - string array of created rigid body names
%       Offsets - corresponding offsets (Nx2)

arguments
    robot (1,1) rigidBodyTree
    options.CornerOffsets (:,2) double
    options.EdgeOffsets (:,2) double
    options.Prefix (1,1) string = "footprint"
    options.Height (1,1) double = 0.0
end

cornerOffsets = options.CornerOffsets;
edgeOffsets = options.EdgeOffsets;
prefix = options.Prefix;
height = options.Height;

allOffsets = [cornerOffsets; edgeOffsets];
nPoints = size(allOffsets, 1);
names = strings(nPoints, 1);

for k = 1:nPoints
    bodyName = sprintf('%s_%d', prefix, k);
    jointName = sprintf('%s_joint_%d', prefix, k);

    body = rigidBody(bodyName);
    joint = rigidBodyJoint(jointName, 'fixed');
    T = trvec2tform([allOffsets(k, :), height]);
    setFixedTransform(joint, T);
    body.Joint = joint;

    addBody(robot, body, robot.BaseName);
    names(k) = string(bodyName);
end

info = struct();
info.Names = names;
info.Offsets = allOffsets;
end
