function bundle = createGikSolver(robot, options)
%CREATEGIKSOLVER Configure generalized inverse kinematics for the robot.
%   bundle = GIK9DOF.CREATEGIKSOLVER(robot) instantiates a
%   generalizedInverseKinematics object together with commonly used
%   constraint handles. The returned struct contains the solver, constraint
%   objects, and helper methods to update targets and weights in a uniform
%   manner.
%
%   bundle = GIK9DOF.CREATEGIKSOLVER(robot, options) supports name-value
%   pairs:
%       EndEffector          - End-effector body name (default
%                              'left_gripper_link').
%       PrimaryTarget        - 4x4 homogeneous transform used to initialise
%                              the pose target constraint (default eye(4)).
%       PrimaryWeights       - 1x2 vector of weights for translation and
%                              orientation components (default [1 1]).
%       JointBoundsWeight    - Scalar weight for constraintJointBounds
%                              (default 0.01).
%       SolverAlgorithm      - Solver algorithm string passed to GIK
%                              (default 'LevenbergMarquardt').
%       EnableAiming         - Set true to create a constraintAiming object.
%       AimTarget            - 1x3 vector (default [0 0 1]) used when aiming
%                              is enabled; interpreted in world coordinates.
%       AimWeight            - Scalar weight for aiming constraint (default
%                              0.1).
%       DistanceBody         - Body that should maintain a distance margin.
%                              Leave empty to skip the distance constraint.
%       DistanceReferenceBody- Reference body used by distance constraint
%                              (default robot.BaseName).
%       DistanceBounds       - 1x2 vector [lower upper] distance bounds. Use
%                              Inf for no upper limit (default [0.2 Inf]).
%       DistanceWeight       - Scalar weight for distance constraint
%                              (default 0.5).
%
%   Example:
%       robot = gik9dof.createRobotModel();
%       bundle = gik9dof.createGikSolver(robot, "EnableAiming", true);
%       q = bundle.tools.column(bundle.tools.home());
%       [qNext, info] = bundle.solve(q, "TargetPose", trvec2tform([0.6 0.2 1.0]));
%
%   See also generalizedInverseKinematics, constraintPoseTarget,
%   constraintJointBounds, constraintDistanceBounds, constraintAiming.

arguments
    robot (1,1) rigidBodyTree
    options.EndEffector (1,1) string = "left_gripper_link"
    options.PrimaryTarget (4,4) double = eye(4)
    options.PrimaryWeights (1,2) double = [1 1]
    options.JointBoundsWeight (1,1) double = 0.01
    options.SolverAlgorithm (1,1) string = "LevenbergMarquardt"
    options.EnableAiming (1,1) logical = false
    options.AimTarget (1,3) double = [0 0 1]
    options.AimWeight (1,1) double = 0.1
    options.DistanceBody (1,1) string = ""
    options.DistanceReferenceBody (1,1) string = ""
    options.DistanceBounds (1,2) double = [0.2 Inf]
    options.DistanceWeight (1,1) double = 0.5
    options.DistanceSpecs = struct([])
end

% Keep a configuration toolbox around for conversions.
configTools = gik9dof.configurationTools(robot);
nJoints = numel(configTools.homeConfig());

% Primary pose tracking task.
poseTgt = constraintPoseTarget(options.EndEffector);
poseTgt.TargetTransform = options.PrimaryTarget;
poseTgt.Weights = options.PrimaryWeights;

% Joint limit constraint for soft enforcement.
jointConst = constraintJointBounds(robot);
jointConst.Weights = options.JointBoundsWeight * ones(1, nJoints);

% Optional aiming constraint.
constraintInputs = {'pose', 'joint'};

if options.EnableAiming
    aimConst = constraintAiming(options.EndEffector);
    aimConst.TargetPoint = options.AimTarget;
    aimConst.Weights = options.AimWeight;
    constraintInputs{end+1} = 'aim'; %#ok<AGROW>
else
    aimConst = [];
end

distanceSpecs = buildDistanceSpecs(robot, options);
distanceConst = cell(1, numel(distanceSpecs));
for i = 1:numel(distanceSpecs)
    spec = distanceSpecs(i);
    dc = constraintDistanceBounds(char(spec.Body));
    dc.ReferenceBody = char(spec.ReferenceBody);
    dc.Bounds = spec.Bounds;
    dc.Weights = spec.Weight;
    distanceConst{i} = dc;
end

if ~isempty(distanceConst)
    constraintInputs = [constraintInputs, repmat({'distance'}, 1, numel(distanceConst))]; %#ok<AGROW>
end

% Configure core solver now that constraint sequence is known.
gik = generalizedInverseKinematics( ...
    'RigidBodyTree', robot, ...
    'SolverAlgorithm', options.SolverAlgorithm, ...
    'ConstraintInputs', constraintInputs);

% Package solver bundle.
bundle.solver = gik;
bundle.constraints.pose = poseTgt;
bundle.constraints.joint = jointConst;
bundle.constraints.aim = aimConst;
bundle.constraints.distance = distanceConst;
bundle.distanceSpecs = distanceSpecs;

bundle.tools.home = @() configTools.homeConfig();
bundle.tools.column = @(q) configTools.column(q);
bundle.tools.struct = @(q) configTools.struct(q);
bundle.tools.random = @() configTools.random();

bundle.updatePrimaryTarget = @(target) setPrimaryTarget(poseTgt, target);
bundle.setPrimaryWeights = @(weights) setPrimaryWeights(poseTgt, weights);
bundle.setJointBoundsWeight = @(w) setJointWeight(jointConst, w);
bundle.setAimTarget = @(point) setAimTarget(aimConst, point);
bundle.setAimWeight = @(w) setAimWeight(aimConst, w);
bundle.setDistanceBounds = @(bounds, idx) setDistanceBounds(distanceConst, bounds, idx);
bundle.setDistanceReference = @(ref, idx) setDistanceReference(distanceConst, ref, idx);
bundle.setDistanceWeight = @(w, idx) setDistanceWeight(distanceConst, w, idx);

bundle.solve = @(q0, varargin) solveStep(gik, q0, poseTgt, jointConst, aimConst, distanceConst, varargin{:});
end

function setPrimaryTarget(poseTgt, target)
if isempty(poseTgt)
    return
end
validateattributes(target, {'double'}, {'size', [4 4], 'finite', 'real'}, mfilename, 'target');
poseTgt.TargetTransform = target;
end

function setPrimaryWeights(poseTgt, weights)
if isempty(poseTgt)
    return
end
validateattributes(weights, {'double'}, {'size', [1 2], 'nonnegative'}, mfilename, 'weights');
poseTgt.Weights = weights;
end

function setJointWeight(jointConst, weight)
if isempty(jointConst)
    return
end
validateattributes(weight, {'double'}, {'scalar', 'nonnegative'}, mfilename, 'weight');
jointConst.Weights = weight;
end

function setAimTarget(aimConst, point)
if isempty(aimConst)
    return
end
validateattributes(point, {'double'}, {'numel', 3, 'finite', 'real'}, mfilename, 'point');
aimConst.TargetPoint = reshape(point, 1, []);
end

function setAimWeight(aimConst, weight)
if isempty(aimConst)
    return
end
validateattributes(weight, {'double'}, {'scalar', 'nonnegative'}, mfilename, 'weight');
aimConst.Weights = weight;
end

function setDistanceBounds(distanceConst, bounds, idx)
if isempty(distanceConst) || isempty(bounds)
    return
end
idxList = resolveDistanceIndices(distanceConst, idx);
bounds = double(bounds);
if isvector(bounds)
    boundsRow = reshape(bounds, 1, []);
    if numel(boundsRow) ~= 2
        error("gik9dof:createGikSolver:InvalidDistanceBounds", ...
            "Bounds must contain exactly two elements.");
    end
    boundsMatrix = repmat(boundsRow, numel(idxList), 1);
elseif size(bounds, 1) == 2 && size(bounds, 2) == numel(idxList)
    boundsMatrix = bounds.';
else
    error("gik9dof:createGikSolver:InvalidDistanceBounds", ...
        "Bounds must be a 1x2 vector or 2xN matrix matching the number of constraints.");
end

boundsMatrix(~isfinite(boundsMatrix)) = realmax;

if any(boundsMatrix(:,2) <= boundsMatrix(:,1))
    error("gik9dof:createGikSolver:InvalidDistanceBounds", ...
        "Upper distance bound must be greater than lower bound for all constraints.");
end

for k = 1:numel(idxList)
    distanceConst{idxList(k)}.Bounds = boundsMatrix(k, :);
end
end

function setDistanceReference(distanceConst, referenceBody, idx)
if isempty(distanceConst) || isempty(referenceBody)
    return
end
idxList = resolveDistanceIndices(distanceConst, idx);

if ischar(referenceBody) || (isstring(referenceBody) && isscalar(referenceBody))
    refs = repmat(string(referenceBody), numel(idxList), 1);
elseif isstring(referenceBody) || iscellstr(referenceBody)
    refs = string(referenceBody);
    if numel(refs) ~= numel(idxList)
        error("gik9dof:createGikSolver:ReferenceCountMismatch", ...
            "Number of reference bodies must match the number of selected constraints.");
    end
else
    error("gik9dof:createGikSolver:InvalidReferenceBody", ...
        "Reference body must be provided as string(s) or char arrays.");
end

for k = 1:numel(idxList)
    distanceConst{idxList(k)}.ReferenceBody = char(refs(k));
end
end

function setDistanceWeight(distanceConst, weight, idx)
if isempty(distanceConst) || isempty(weight)
    return
end
idxList = resolveDistanceIndices(distanceConst, idx);
weight = double(weight);
if isscalar(weight)
    weights = repmat(weight, 1, numel(idxList));
elseif numel(weight) == numel(idxList)
    weights = reshape(weight, 1, []);
else
    error("gik9dof:createGikSolver:WeightCountMismatch", ...
        "Number of weights must match the number of selected constraints.");
end

if any(weights < 0)
    error("gik9dof:createGikSolver:InvalidDistanceWeight", ...
        "Weights must be non-negative.");
end

for k = 1:numel(idxList)
    distanceConst{idxList(k)}.Weights = weights(k);
end
end

function idxList = resolveDistanceIndices(distanceConst, idx)
if isempty(distanceConst)
    idxList = [];
    return
end
if nargin < 2 || isempty(idx)
    idxList = 1:numel(distanceConst);
else
    idxList = unique(double(idx(:)'));
    if any(idxList < 1) || any(idxList > numel(distanceConst))
        error("gik9dof:createGikSolver:DistanceIndexOutOfRange", ...
            "Distance constraint index out of range.");
    end
end
end

function specs = buildDistanceSpecs(robot, options)
specs = struct([]);

if ~isempty(options.DistanceSpecs)
    if ~isstruct(options.DistanceSpecs)
        error("gik9dof:createGikSolver:InvalidDistanceSpecs", ...
            "DistanceSpecs must be a struct array.");
    end
    specs = options.DistanceSpecs;
end

if strlength(options.DistanceBody) > 0
    spec.Body = options.DistanceBody;
    if strlength(options.DistanceReferenceBody) > 0
        spec.ReferenceBody = options.DistanceReferenceBody;
    else
        spec.ReferenceBody = robot.BaseName;
    end
    spec.Bounds = options.DistanceBounds;
    spec.Weight = options.DistanceWeight;
    specs = [specs(:); spec]; %#ok<AGROW>
end

if isempty(specs)
    return
end

defaultBounds = options.DistanceBounds;
defaultWeight = options.DistanceWeight;

normalized = repmat(struct('Body', "", ...
    'ReferenceBody', "", 'Bounds', [0.2, Inf], 'Weight', defaultWeight), numel(specs), 1);

for i = 1:numel(specs)
    entry = specs(i);
    if ~isfield(entry, 'Body')
        error("gik9dof:createGikSolver:MissingDistanceBody", ...
            "Distance spec %d is missing the 'Body' field.", i);
    end
    bodyName = string(entry.Body);
    if strlength(bodyName) == 0
        error("gik9dof:createGikSolver:EmptyDistanceBody", ...
            "Distance spec %d has an empty body name.", i);
    end
    validateBodyPresence(robot, bodyName, "Body");

    if isfield(entry, 'ReferenceBody') && strlength(string(entry.ReferenceBody)) > 0
        refName = string(entry.ReferenceBody);
    else
        refName = string(robot.BaseName);
    end
    validateBodyPresence(robot, refName, "ReferenceBody");

    if isfield(entry, 'Bounds') && ~isempty(entry.Bounds)
        bounds = double(entry.Bounds);
    else
        bounds = defaultBounds;
    end
    bounds = reshape(bounds, 1, []);
    if numel(bounds) ~= 2
        error("gik9dof:createGikSolver:InvalidDistanceBounds", ...
            "Distance spec %d must provide bounds with two elements.", i);
    end
    bounds(~isfinite(bounds)) = realmax;
    if bounds(2) <= bounds(1)
        error("gik9dof:createGikSolver:InvalidDistanceBounds", ...
            "Distance spec %d has upper bound <= lower bound.", i);
    end

    if isfield(entry, 'Weight') && ~isempty(entry.Weight)
        weight = double(entry.Weight);
    else
        weight = defaultWeight;
    end
    if weight < 0
        error("gik9dof:createGikSolver:InvalidDistanceWeight", ...
            "Distance spec %d has a negative weight.", i);
    end

    normalized(i).Body = bodyName;
    normalized(i).ReferenceBody = refName;
    normalized(i).Bounds = bounds;
    normalized(i).Weight = weight;
end

specs = normalized;
end

function validateBodyPresence(robot, bodyName, role)
try
    robot.getBody(char(bodyName));
catch
    error("gik9dof:createGikSolver:UnknownBody", ...
        "%s '%s' not present in robot model.", role, bodyName);
end
end

function [qNext, info] = solveStep(gik, q0, poseTgt, jointConst, aimConst, distanceConst, varargin)
%SOLVESTEP Wrapper that updates constraint targets before invoking GIK.
parser = inputParser;
parser.FunctionName = 'gik9dof:createGikSolver';
addParameter(parser, 'TargetPose', poseTgt.TargetTransform, @(x) isnumeric(x) && isequal(size(x), [4 4]));
addParameter(parser, 'AimTarget', [], @(x) isempty(x) || (isnumeric(x) && numel(x) == 3));
addParameter(parser, 'DistanceBounds', [], @(x) isempty(x) || (isnumeric(x) && (numel(x) == 2 || size(x,1) == 2 || size(x,2) == 2)));
addParameter(parser, 'DistanceReference', [], @(x) isempty(x) || ischar(x) || iscellstr(x) || isstring(x));
parse(parser, varargin{:});
params = parser.Results;

poseTgt.TargetTransform = params.TargetPose;

if ~isempty(aimConst) && ~isempty(params.AimTarget)
    aimConst.TargetPoint = reshape(params.AimTarget, 1, []);
end

if ~isempty(distanceConst)
    if ~isempty(params.DistanceBounds)
        setDistanceBounds(distanceConst, params.DistanceBounds);
    end
    if ~isempty(params.DistanceReference)
        setDistanceReference(distanceConst, params.DistanceReference);
    end
end

constraints = {poseTgt, jointConst};
if ~isempty(aimConst)
    constraints{end+1} = aimConst; %#ok<AGROW>
end
if ~isempty(distanceConst)
    constraints = [constraints, distanceConst]; %#ok<*AGROW>
end

[qNext, info] = gik(q0, constraints{:});
end
