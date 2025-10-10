function params = createDefaultChassisPathParams(chassisProfile)
%CREATEDEFAULTCHASSISPATHPARAMS Create default parameters for chassisPathFollowerCodegen
%
%   params = createDefaultChassisPathParams(chassisProfile) creates a parameter
%   struct with default values for the chassis path follower. This documents
%   all 30+ parameters and provides sensible defaults for tuning.
%
% INPUTS:
%   chassisProfile - (optional) struct with chassis physical parameters:
%                    • track: wheel track width (m)
%                    • wheel_speed_max: maximum wheel speed (m/s)
%                    • vx_max: maximum forward speed (m/s)
%                    • vx_min: minimum forward speed (m/s, typically negative)
%                    • wz_max: maximum angular velocity (rad/s)
%                    • accel_limit: acceleration limit (m/s²)
%                    • decel_limit: deceleration limit (m/s²)
%                    • jerk_limit: jerk limit (m/s³)
%                    • wheel_base: distance between front and rear axles (m)
%                    • reverse_enabled: allow reverse motion (boolean)
%                    If omitted, uses default values for typical mobile manipulator
%
% OUTPUTS:
%   params - complete parameter struct for chassisPathFollowerCodegen with fields:
%            ┌─ Controller Selection
%            │  • ControllerMode: 'blended' (or 'purePursuit'/'stanley')
%            │
%            ├─ Lookahead Tuning (Adaptive)
%            │  • LookaheadBase: base lookahead distance (m)
%            │  • LookaheadVelGain: velocity-dependent gain (s)
%            │  • LookaheadAccelGain: acceleration-dependent gain (s²/m)
%            │
%            ├─ Goal Detection
%            │  • GoalTolerance: distance threshold for goal (m)
%            │
%            ├─ Heading Control (PID)
%            │  • HeadingKp: proportional gain
%            │  • HeadingKi: integral gain
%            │  • HeadingKd: derivative gain
%            │  • FeedforwardGain: yaw rate feedforward (0-1)
%            │
%            ├─ Chassis Model
%            │  • Chassis_track: wheel track (m)
%            │  • Chassis_wheel_speed_max: max wheel speed (m/s)
%            │  • Chassis_vx_max: max forward speed (m/s)
%            │  • Chassis_vx_min: min forward speed (m/s)
%            │  • Chassis_wz_max: max angular velocity (rad/s)
%            │  • Chassis_accel_limit: acceleration limit (m/s²)
%            │  • Chassis_decel_limit: deceleration limit (m/s²)
%            │  • Chassis_jerk_limit: jerk limit (m/s³)
%            │  • Chassis_wheel_base: wheelbase (m)
%            │  • Chassis_reverse_enabled: reverse allowed (boolean)
%            │
%            ├─ Curvature-Based Speed Control
%            │  • CurvatureSlowdown_kappa_threshold: curvature threshold (rad/m)
%            │  • CurvatureSlowdown_vx_reduction: speed reduction fraction (0-1)
%            │
%            └─ Path Information (Preprocessed)
%               • PathInfo_States: Nx3 [x y theta]
%               • PathInfo_Curvature: Nx1 curvature (rad/m)
%               • PathInfo_ArcLength: Nx1 cumulative arc length (m)
%               • PathInfo_DistanceRemaining: Nx1 distance to goal (m)
%
% PARAMETER TUNING GUIDE:
%
%   LOOKAHEAD TUNING:
%   ─────────────────
%   LookaheadBase (m):
%     • 0.4-0.6: Tight spaces, slow speeds, aggressive tracking
%     • 0.6-0.8: Balanced performance (DEFAULT: 0.6)
%     • 0.8-1.2: High speeds, smooth paths, gentle tracking
%
%   LookaheadVelGain (s):
%     • 0.2: Less adaptive, stays close to base
%     • 0.3: Moderate adaptation (DEFAULT)
%     • 0.4-0.5: Highly adaptive, grows quickly with speed
%
%   LookaheadAccelGain (s²/m):
%     • 0.0: No acceleration compensation
%     • 0.05: Moderate (DEFAULT)
%     • 0.1: Strong compensation for acceleration
%
%   HEADING CONTROL:
%   ───────────────
%   HeadingKp:
%     • 0.8-1.0: Gentle heading correction
%     • 1.2: Balanced (DEFAULT)
%     • 1.5-2.0: Aggressive heading correction
%
%   HeadingKi:
%     • 0.0: No integral term (DEFAULT, recommended)
%     • 0.1-0.5: Eliminate steady-state heading error (use with caution)
%
%   HeadingKd:
%     • 0.0: No derivative term
%     • 0.1: Light damping (DEFAULT)
%     • 0.2-0.3: Strong damping (reduces oscillations)
%
%   FeedforwardGain:
%     • 0.7-0.8: Conservative feedforward
%     • 0.9: Balanced (DEFAULT)
%     • 1.0: Full feedforward (use if path curvature is accurate)
%
%   CHASSIS LIMITS:
%   ──────────────
%   accel_limit (m/s²):
%     • 0.5-0.8: Very smooth, gentle (e.g., carrying payload)
%     • 1.0-1.2: Balanced performance (DEFAULT: 1.2)
%     • 1.5-2.0: Aggressive, sporty
%
%   decel_limit (m/s²):
%     • 0.8-1.2: Gentle braking
%     • 1.5-1.8: Moderate braking (DEFAULT: 1.8)
%     • 2.0-3.0: Emergency stop capable
%
%   jerk_limit (m/s³):
%     • 2.0-3.0: Very smooth (e.g., liquid transport)
%     • 4.0-6.0: Smooth normal operation (DEFAULT: 5.0)
%     • 8.0-10.0: Responsive, less smooth
%
%   CURVATURE CONTROL:
%   ─────────────────
%   kappa_threshold (rad/m):
%     • 0.5: Start slowing early (gentle curves)
%     • 0.9: Balanced (DEFAULT)
%     • 1.5: Only slow in very sharp curves
%
%   vx_reduction (fraction):
%     • 0.4-0.5: Aggressive slowdown (50-60% speed)
%     • 0.6: Moderate slowdown (DEFAULT, 60% speed)
%     • 0.7-0.8: Gentle slowdown (70-80% speed)
%
% TYPICAL PARAMETER SETS:
%
%   CONSERVATIVE (Smooth, Safe):
%   ──────────────────────────
%   params.LookaheadBase = 0.8;
%   params.LookaheadVelGain = 0.25;
%   params.HeadingKp = 1.0;
%   params.Chassis_accel_limit = 0.8;
%   params.Chassis_jerk_limit = 3.0;
%   params.CurvatureSlowdown_kappa_threshold = 0.7;
%   params.CurvatureSlowdown_vx_reduction = 0.5;
%
%   BALANCED (Default):
%   ──────────────────
%   params.LookaheadBase = 0.6;
%   params.LookaheadVelGain = 0.3;
%   params.HeadingKp = 1.2;
%   params.Chassis_accel_limit = 1.2;
%   params.Chassis_jerk_limit = 5.0;
%   params.CurvatureSlowdown_kappa_threshold = 0.9;
%   params.CurvatureSlowdown_vx_reduction = 0.6;
%
%   AGGRESSIVE (Fast, Responsive):
%   ─────────────────────────────
%   params.LookaheadBase = 0.5;
%   params.LookaheadVelGain = 0.4;
%   params.HeadingKp = 1.8;
%   params.Chassis_accel_limit = 1.8;
%   params.Chassis_jerk_limit = 8.0;
%   params.CurvatureSlowdown_kappa_threshold = 1.2;
%   params.CurvatureSlowdown_vx_reduction = 0.7;
%
% EXAMPLE USAGE:
%   % Method 1: Use defaults
%   params = createDefaultChassisPathParams();
%
%   % Method 2: Provide chassis profile
%   chassisProfile = loadChassisProfile('mobile_manipulator');
%   params = createDefaultChassisPathParams(chassisProfile);
%
%   % Method 3: Override specific parameters
%   params = createDefaultChassisPathParams();
%   params.ControllerMode = 'purePursuit';  % Change mode
%   params.LookaheadBase = 0.8;             % Increase lookahead
%   params.HeadingKp = 1.5;                 % More aggressive heading
%
% CODEGEN NOTE:
%   This function flattens nested structs into scalar fields for codegen
%   compatibility. The original class used nested structs:
%       params.Chassis.track
%       params.Chassis.curvature_slowdown.kappa_threshold
%   
%   Now flattened to:
%       params.Chassis_track
%       params.CurvatureSlowdown_kappa_threshold
%
% See also: chassisPathFollowerCodegen, preparePathForFollower,
%           initializeChassisPathState

%% Handle Optional Input
if nargin < 1 || isempty(chassisProfile)
    % Default chassis parameters for typical mobile manipulator
    chassisProfile = struct();
    chassisProfile.track = 0.573;              % m (Clearpath Ridgeback)
    chassisProfile.wheel_speed_max = 3.3;      % m/s
    chassisProfile.vx_max = 1.5;               % m/s
    chassisProfile.vx_min = -0.4;              % m/s (reverse)
    chassisProfile.wz_max = 2.5;               % rad/s
    chassisProfile.accel_limit = 1.2;          % m/s²
    chassisProfile.decel_limit = 1.8;          % m/s²
    chassisProfile.jerk_limit = 5.0;           % m/s³
    chassisProfile.wheel_base = 0.36;          % m
    chassisProfile.reverse_enabled = false;    % typically false for safety
end

%% Create Parameters Struct
params = struct();

% ──────────────────────────────────────────────────────────────────────
% Controller Selection
% ──────────────────────────────────────────────────────────────────────
% Mode 0: Legacy 5-point differentiation (open-loop replay)
% Mode 1: Heading-aware controller (simple feedback)
% Mode 2: Pure pursuit (full feedback with advanced features) - DEFAULT
params.ControllerMode = 2;  % 0, 1, or 2

% ──────────────────────────────────────────────────────────────────────
% Reverse Handling
% ──────────────────────────────────────────────────────────────────────
params.ReverseEnabled = chassisProfile.reverse_enabled;  % Allow reverse motion

% ──────────────────────────────────────────────────────────────────────
% Lookahead Tuning (Adaptive - Used in Modes 1 and 2)
% ──────────────────────────────────────────────────────────────────────
% Total lookahead = LookaheadBase + LookaheadVelGain * |vx| + LookaheadAccelGain * accel
params.LookaheadBase = 0.6;          % m - Base lookahead distance
params.LookaheadVelGain = 0.30;      % s - Velocity-proportional gain
params.LookaheadAccelGain = 0.05;    % s²/m - Acceleration-proportional gain

% ──────────────────────────────────────────────────────────────────────
% Goal Detection
% ──────────────────────────────────────────────────────────────────────
params.GoalTolerance = 0.10;         % m - Distance threshold for goal reached

% ──────────────────────────────────────────────────────────────────────
% Heading Control (PID - Used in Mode 1)
% ──────────────────────────────────────────────────────────────────────
params.HeadingKp = 1.2;              % Proportional gain (higher = more aggressive)
params.HeadingKi = 0.0;              % Integral gain (usually keep at 0)
params.HeadingKd = 0.1;              % Derivative gain (damping)
params.FeedforwardGain = 0.9;        % Feedforward gain for path yaw rate (0-1)

% ──────────────────────────────────────────────────────────────────────
% Curvature-Based Speed Control (Used in Mode 2)
% ──────────────────────────────────────────────────────────────────────
params.KappaThreshold = 0.9;    % rad/m - Curvature threshold for speed reduction
params.VxReduction = 0.6;       % fraction - Speed multiplier in high-curvature (60%)

% ──────────────────────────────────────────────────────────────────────
% Chassis Physical Parameters (Flattened from nested struct for codegen)
% ──────────────────────────────────────────────────────────────────────
params.Chassis = struct();
params.Chassis.track = chassisProfile.track;
params.Chassis.wheel_speed_max = chassisProfile.wheel_speed_max;
params.Chassis.vx_max = chassisProfile.vx_max;
params.Chassis.vx_min = chassisProfile.vx_min;
params.Chassis.wz_max = chassisProfile.wz_max;
params.Chassis.accel_limit = chassisProfile.accel_limit;
params.Chassis.decel_limit = chassisProfile.decel_limit;
params.Chassis.jerk_limit = chassisProfile.jerk_limit;
params.Chassis.wheel_base = chassisProfile.wheel_base;
params.Chassis.reverse_enabled = chassisProfile.reverse_enabled;

% ──────────────────────────────────────────────────────────────────────
% Path Information (Empty, Must Be Set from preparePathForFollower)
% ──────────────────────────────────────────────────────────────────────
% NOTE: These fields must be populated by calling preparePathForFollower
%       before using chassisPathFollowerCodegen. Example:
%
%       PathInfo = preparePathForFollower(waypoints, chassisProfile);
%       params.PathInfo_States = PathInfo.States;
%       params.PathInfo_Curvature = PathInfo.Curvature;
%       params.PathInfo_ArcLength = PathInfo.ArcLength;
%       params.PathInfo_DistanceRemaining = PathInfo.DistanceRemaining;

params.PathInfo_States = zeros(0, 3);              % Nx3 [x y theta] (empty initially)
params.PathInfo_Curvature = zeros(0, 1);           % Nx1 curvature (rad/m)
params.PathInfo_ArcLength = zeros(0, 1);           % Nx1 arc length (m)
params.PathInfo_DistanceRemaining = zeros(0, 1);   % Nx1 distance remaining (m)

%% Validation
% Check for required fields in chassisProfile
requiredFields = {'track', 'wheel_speed_max', 'vx_max', 'vx_min', 'wz_max', ...
    'accel_limit', 'decel_limit', 'jerk_limit', 'wheel_base', 'reverse_enabled'};

for i = 1:numel(requiredFields)
    fieldName = requiredFields{i};
    if ~isfield(chassisProfile, fieldName)
        error('createDefaultChassisPathParams:MissingField', ...
            'Chassis profile missing required field: %s', fieldName);
    end
end

end % createDefaultChassisPathParams
