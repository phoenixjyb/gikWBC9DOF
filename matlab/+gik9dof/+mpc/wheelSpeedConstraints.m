function cineq = wheelSpeedConstraints(X, U, data, params)
% WHEELSPEEDCONSTRAINTS Custom inequality constraints for differential drive
%
% Implements wheel speed limits for a differential drive robot:
%   v_left  = v - ω * W/2
%   v_right = v + ω * W/2
% where W is track width (distance between wheels).
%
% Constraint: |v_left|, |v_right| ≤ v_wheel_max
%
% Inputs:
%   X      - [nx x (p+1)] Predicted state trajectory (not used here)
%   U      - [nu x p] Predicted control trajectory [v; omega; q_dot_arm(6)]
%   data   - Additional data from nlmpcmove (not used here)
%   params - struct with fields:
%            .track_width: Distance between wheels (m)
%            .wheel_max: Maximum wheel speed (m/s)
%
% Output:
%   cineq - [2*p x 1] Constraint vector (must satisfy cineq ≤ 0)
%
% For nlmpc: nlobj.Optimization.CustomIneqConFcn = @(X,U,data) ...
%                wheelSpeedConstraints(X, U, data, params);
%
% See also: nlmpc

% Author: GitHub Copilot + User
% Date: October 13, 2025
% Method: Method 5 (pureMPC) - Whole-body MPC

%% Extract parameters
W = params.track_width;
v_wheel_max = params.wheel_max;

%% Number of prediction steps
p = size(U, 2);

%% Initialize constraint vector
% Each step has 2 constraints (left and right wheel)
cineq = zeros(2*p, 1);

%% Apply constraints for each prediction step
for k = 1:p
    % Extract base velocities (first 2 inputs)
    v = U(1, k);
    omega = U(2, k);
    % Note: U(3:8, k) are arm joint velocities, not used for wheel constraints
    
    % Compute individual wheel speeds
    v_left = v - omega * W/2;
    v_right = v + omega * W/2;
    
    % Constraint: |v_wheel| ≤ v_wheel_max
    % Reformulated as: |v_wheel| - v_wheel_max ≤ 0
    cineq(2*k-1) = abs(v_left) - v_wheel_max;   % Left wheel
    cineq(2*k) = abs(v_right) - v_wheel_max;    % Right wheel
end

end
