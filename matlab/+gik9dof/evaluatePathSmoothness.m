function metrics = evaluatePathSmoothness(path3D, dt)
%EVALUATEPATHSMOOTHNESS Quantify jerkiness and kinks in 3D trajectory.
%   metrics = evaluatePathSmoothness(path3D, dt) analyzes a 3xN trajectory
%   and computes smoothness metrics based on acceleration and jerk.
%
%   Inputs:
%       path3D - 3xN array of [x; y; z] positions
%       dt     - Time step between waypoints [s]
%
%   Outputs:
%       metrics.maxAcceleration    - Peak acceleration magnitude [m/s²]
%       metrics.rmsAcceleration    - RMS acceleration [m/s²]
%       metrics.maxJerk            - Peak jerk magnitude [m/s³]
%       metrics.rmsJerk            - RMS jerk [m/s³]
%       metrics.kinkCount          - Number of sharp direction changes
%       metrics.kinkIndices        - Indices where kinks occur
%       metrics.kinkAngles         - Angles at each kink [deg]
%       metrics.smoothnessScore    - Overall smoothness (0=kinky, 1=smooth)
%       metrics.accelerationProfile - Nx1 acceleration magnitudes
%       metrics.jerkProfile         - Nx1 jerk magnitudes

arguments
    path3D (3,:) double
    dt (1,1) double = 0.1
end

N = size(path3D, 2);
metrics = struct();

if N < 3
    % Not enough points
    metrics.maxAcceleration = 0;
    metrics.rmsAcceleration = 0;
    metrics.maxJerk = 0;
    metrics.rmsJerk = 0;
    metrics.kinkCount = 0;
    metrics.kinkIndices = [];
    metrics.kinkAngles = [];
    metrics.smoothnessScore = 1.0;
    metrics.accelerationProfile = zeros(N, 1);
    metrics.jerkProfile = zeros(N, 1);
    return;
end

% Compute velocity (1st derivative)
velocity = diff(path3D, 1, 2) / dt;  % 3x(N-1)

% Compute acceleration (2nd derivative)
acceleration = diff(velocity, 1, 2) / dt;  % 3x(N-2)
accelMagnitude = vecnorm(acceleration, 2, 1)';  % (N-2)x1

% Compute jerk (3rd derivative)
jerk = diff(acceleration, 1, 2) / dt;  % 3x(N-3)
jerkMagnitude = vecnorm(jerk, 2, 1)';  % (N-3)x1

% Pad to match original length
accelProfile = [0; 0; accelMagnitude; accelMagnitude(end)];
jerkProfile = [0; 0; 0; jerkMagnitude; jerkMagnitude(end); jerkMagnitude(end)];

% Acceleration statistics
maxAcceleration = max(accelMagnitude);
rmsAcceleration = sqrt(mean(accelMagnitude.^2));

% Jerk statistics
maxJerk = max(jerkMagnitude);
rmsJerk = sqrt(mean(jerkMagnitude.^2));

% Detect kinks: sharp angle changes in velocity direction
% Compute angle between consecutive velocity segments
kinkIndices = [];
kinkAngles = [];
kinkThreshold = 30;  % degrees

speedProfile = vecnorm(velocity, 2, 1);
for i = 2:size(velocity, 2)
    % Check if both segments have sufficient speed
    if speedProfile(i-1) > 0.01 && speedProfile(i) > 0.01
        % Normalize velocity vectors
        v1 = velocity(:, i-1) / speedProfile(i-1);
        v2 = velocity(:, i) / speedProfile(i);
        
        % Angle between vectors
        cosAngle = dot(v1, v2);
        cosAngle = max(-1, min(1, cosAngle));  % Clamp to [-1, 1]
        angleDeg = rad2deg(acos(cosAngle));
        
        % Check if it's a kink
        if angleDeg > kinkThreshold
            kinkIndices = [kinkIndices; i]; %#ok<AGROW>
            kinkAngles = [kinkAngles; angleDeg]; %#ok<AGROW>
        end
    end
end

kinkCount = length(kinkIndices);

% Compute overall smoothness score
% Based on normalized RMS jerk and kink count
% Lower jerk and fewer kinks = higher score
jerkScore = exp(-rmsJerk / 10);  % Normalize: jerk ~10 m/s³ -> score ~0.37
kinkPenalty = exp(-kinkCount / 5);  % 5 kinks -> score ~0.37
smoothnessScore = 0.6 * jerkScore + 0.4 * kinkPenalty;

% Populate output
metrics.maxAcceleration = maxAcceleration;
metrics.rmsAcceleration = rmsAcceleration;
metrics.maxJerk = maxJerk;
metrics.rmsJerk = rmsJerk;
metrics.kinkCount = kinkCount;
metrics.kinkIndices = kinkIndices;
metrics.kinkAngles = kinkAngles;
metrics.smoothnessScore = smoothnessScore;
metrics.accelerationProfile = accelProfile;
metrics.jerkProfile = jerkProfile;

end
