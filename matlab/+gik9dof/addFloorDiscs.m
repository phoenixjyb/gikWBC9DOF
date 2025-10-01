function discsOut = addFloorDiscs(robot, discs, options)
%ADDFLOORDISCS Insert fixed frames for floor obstacles into the robot tree.
%   discsOut = gik9dof.addFloorDiscs(robot, discs) attaches each disc as a
%   fixed body under the robot base. The input discs must be a struct array
%   with fields:
%       Center        - 1x2 [x y] position in metres (required).
%       Radius        - Scalar radius in metres (required, >0).
%       Name          - Optional string/char name for the frame.
%       SafetyMargin  - Optional scalar additional clearance (>=0).
%
%   discsOut mirrors the input order and reports the resolved names,
%   radii, and safety margins used for distance constraints.
%
%   Options:
%       ParentBody - Body name to attach the discs to (default
%                    robot.BaseName).
%       Height     - Scalar z-offset for the disc frame (default 0).
%
arguments
    robot (1,1) rigidBodyTree
    discs (1,:) struct
    options.ParentBody (1,1) string = string(robot.BaseName)
    options.Height (1,1) double = 0
end

if isempty(discs)
    discsOut = struct('Name', {}, 'Radius', {}, 'SafetyMargin', {});
    return
end

parentBody = char(options.ParentBody);
% Validate parent body presence unless it is the base frame.
if ~strcmp(parentBody, robot.BaseName)
    try
        robot.getBody(parentBody);
    catch
        error("gik9dof:addFloorDiscs:UnknownParent", ...
            "Parent body '%s' not present in robot.", parentBody);
    end
end

numDiscs = numel(discs);
resolved = repmat(struct('Name', "", 'Radius', 0, 'SafetyMargin', 0, 'Center', [0 0]), numDiscs, 1);
existingNames = string(robot.BodyNames);

for k = 1:numDiscs
    disc = discs(k);
    if ~isfield(disc, 'Center') || numel(disc.Center) ~= 2
        error("gik9dof:addFloorDiscs:InvalidCenter", ...
            "Disc %d must provide a 1x2 Center.", k);
    end
    if ~isfield(disc, 'Radius') || disc.Radius <= 0
        error("gik9dof:addFloorDiscs:InvalidRadius", ...
            "Disc %d must provide a positive Radius.", k);
    end

    if isfield(disc, 'SafetyMargin') && ~isempty(disc.SafetyMargin)
        safety = disc.SafetyMargin;
        if safety < 0
            error("gik9dof:addFloorDiscs:InvalidSafetyMargin", ...
                "Disc %d safety margin must be non-negative.", k);
        end
    else
        safety = 0;
    end

    if isfield(disc, 'Name') && ~isempty(disc.Name)
        candidateName = string(disc.Name);
    else
        candidateName = sprintf('floor_disc_%d', k);
    end

    % Ensure uniqueness against existing and newly created bodies.
    candidateName = matlab.lang.makeUniqueStrings(candidateName, existingNames);
    existingNames(end+1) = candidateName; %#ok<AGROW>

    body = rigidBody(char(candidateName));
    joint = rigidBodyJoint([char(candidateName) '_joint'], 'fixed');
    transform = trvec2tform([disc.Center(:).', options.Height]);
    setFixedTransform(joint, transform);
    body.Joint = joint;
    addBody(robot, body, parentBody);

    resolved(k).Name = candidateName;
    resolved(k).Radius = disc.Radius;
    resolved(k).SafetyMargin = safety;
    resolved(k).Center = reshape(disc.Center(1:2), 1, 2);
end

discsOut = resolved;
end
