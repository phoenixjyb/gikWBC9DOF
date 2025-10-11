function metrics = evaluateChassisConstraints(baseStates, dt, chassisParams)
%EVALUATECHASSISCONSTRAINTS Validate differential drive constraints.
%   metrics = evaluateChassisConstraints(baseStates, dt, chassisParams)
%   analyzes an Nx3 SE(2) trajectory [x, y, yaw] and detects violations
%   of differential drive kinematics (sideways/diagonal movement).
%
%   Inputs:
%       baseStates    - Nx3 array of [x, y, yaw] poses
%       dt            - Time step between poses [s]
%       chassisParams - Struct with chassis geometry (optional)
%
%   Outputs:
%       metrics.sidewaysViolations     - Count of timesteps with sideways motion
%       metrics.sidewaysViolationPct   - Percentage of trajectory
%       metrics.maxSidewaysVelocity    - Peak lateral velocity [m/s]
%       metrics.meanSidewaysVelocity   - Average lateral velocity [m/s]
%       metrics.alignmentErrors        - Nx1 angle between velocity & heading [rad]
%       metrics.maxAlignmentError      - Peak misalignment [deg]
%       metrics.rmsAlignmentError      - RMS misalignment [deg]
%       metrics.violationIndices       - Indices where violations occur
%       metrics.velocityProfile        - Nx2 [vx_body, vy_body] in body frame

arguments
    baseStates (:,3) double
    dt (1,1) double = 0.1
    chassisParams struct = struct()
end

N = size(baseStates, 1);
metrics = struct();

if N < 2
    % Not enough points
    metrics.sidewaysViolations = 0;
    metrics.sidewaysViolationPct = 0;
    metrics.maxSidewaysVelocity = 0;
    metrics.meanSidewaysVelocity = 0;
    metrics.alignmentErrors = zeros(N, 1);
    metrics.maxAlignmentError = 0;
    metrics.rmsAlignmentError = 0;
    metrics.violationIndices = [];
    metrics.velocityProfile = zeros(N, 2);
    return;
end

% Extract trajectory
x = baseStates(:, 1);
y = baseStates(:, 2);
theta = baseStates(:, 3);

% Compute velocities in world frame
vx_world = diff(x) / dt;
vy_world = diff(y) / dt;
omega = wrapToPi(diff(theta)) / dt;

% Transform velocities to body frame
% Body frame: x-forward, y-left
velocityProfile = zeros(N-1, 2);
alignmentErrors = zeros(N-1, 1);

for i = 1:(N-1)
    % Rotation matrix from world to body frame at pose i
    c = cos(theta(i));
    s = sin(theta(i));
    R_world_to_body = [c, s; -s, c];
    
    % Velocity in body frame
    v_world = [vx_world(i); vy_world(i)];
    v_body = R_world_to_body * v_world;
    
    velocityProfile(i, :) = v_body';
    
    % Alignment error: angle between velocity vector and heading direction
    % For differential drive, velocity should be purely longitudinal (vy_body ≈ 0)
    speed = norm(v_world);
    if speed > 1e-3  % Only check if moving
        % Alignment error is atan2(vy_body, vx_body)
        alignmentErrors(i) = atan2(abs(v_body(2)), abs(v_body(1)));
    else
        alignmentErrors(i) = 0;
    end
end

% Pad last element
velocityProfile = [velocityProfile; velocityProfile(end, :)];
alignmentErrors = [alignmentErrors; alignmentErrors(end)];

% Detect violations
% Threshold: lateral velocity > 10% of longitudinal velocity
% or alignment error > 10 degrees
lateralVel = abs(velocityProfile(:, 2));
longitudinalVel = abs(velocityProfile(:, 1));
speed = sqrt(sum(velocityProfile.^2, 2));

% Define violation criteria
alignmentThreshold = deg2rad(10);  % 10 degrees
lateralRatioThreshold = 0.1;       % 10% lateral component

violations = (alignmentErrors > alignmentThreshold) & (speed > 0.01);

% Count violations
violationIndices = find(violations);
sidewaysViolations = sum(violations);
sidewaysViolationPct = 100 * sidewaysViolations / N;

% Statistics
maxSidewaysVelocity = max(lateralVel);
meanSidewaysVelocity = mean(lateralVel);
maxAlignmentError = rad2deg(max(alignmentErrors));
rmsAlignmentError = rad2deg(sqrt(mean(alignmentErrors.^2)));

% Populate output
metrics.sidewaysViolations = sidewaysViolations;
metrics.sidewaysViolationPct = sidewaysViolationPct;
metrics.maxSidewaysVelocity = maxSidewaysVelocity;
metrics.meanSidewaysVelocity = meanSidewaysVelocity;
metrics.alignmentErrors = alignmentErrors;
metrics.maxAlignmentError = maxAlignmentError;
metrics.rmsAlignmentError = rmsAlignmentError;
metrics.violationIndices = violationIndices;
metrics.velocityProfile = velocityProfile;

end
