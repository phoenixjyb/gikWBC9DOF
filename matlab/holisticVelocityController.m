function [Vx, Wz, stateOut] = holisticVelocityController(...
    refX, refY, refTheta, refTime, ...
    estX, estY, estYaw, ...
    stateIn, ...
    params)
%HOLISTICVELOCITYCONTROLLER Simplified wrapper for codegen of unifiedChassisCtrl
%   [Vx, Wz, stateOut] = holisticVelocityController(
%       refX, refY, refTheta, refTime,     % Target position from IK solver
%       estX, estY, estYaw,                 % Current robot pose estimate
%       stateIn,                            % Controller state (persistent memory)
%       params)                             % Controller parameters
%
%   Wraps gik9dof.control.unifiedChassisCtrl in "holistic" mode with
%   scalar inputs/outputs for easier C++ code generation and ROS2 integration.
%
%   Inputs:
%       refX, refY, refTheta - Target base pose in world frame [m, m, rad]
%       refTime              - Timestamp of reference [s]
%       estX, estY, estYaw   - Estimated robot pose [m, m, rad]
%       stateIn              - State struct (carries prev reference)
%       params               - Parameter struct (track, Vx_max, etc.)
%
%   Outputs:
%       Vx       - Forward velocity command [m/s]
%       Wz       - Yaw rate command [rad/s]
%       stateOut - Updated state struct for next call
%
%   See also gik9dof.control.unifiedChassisCtrl, 
%            gik9dof.control.clampYawByWheelLimit

arguments
    refX (1,1) double
    refY (1,1) double
    refTheta (1,1) double
    refTime (1,1) double
    estX (1,1) double
    estY (1,1) double
    estYaw (1,1) double
    stateIn struct
    params struct
end

% Build reference struct for unifiedChassisCtrl
ref = struct();
ref.x = refX;
ref.y = refY;
ref.theta = refTheta;
ref.t = refTime;

% Current pose estimate as 1x3 vector
estPose = [estX, estY, estYaw];

% Call main controller in "holistic" mode
[cmd, stateOut] = gik9dof.control.unifiedChassisCtrl(...
    "holistic", ref, estPose, stateIn, params);

% Extract velocity commands
Vx = cmd.base.Vx;
Wz = cmd.base.Wz;

end
