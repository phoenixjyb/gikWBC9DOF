function x_next = unicycleStateFcn(x, u, Ts)
% UNICYCLESTATEFCN State transition function for nonlinear MPC
%
% This function defines the discrete-time dynamics of a differential drive
% robot using the unicycle kinematic model.
%
% Inputs:
%   x  - [3x1] Current state [x_pos; y_pos; theta]
%        x_pos: X position in world frame (m)
%        y_pos: Y position in world frame (m)
%        theta: Heading angle (rad)
%   u  - [2x1] Control inputs [v; omega]
%        v: Linear velocity (m/s)
%        omega: Angular velocity (rad/s)
%   Ts - Sample time (s)
%
% Output:
%   x_next - [3x1] Next state [x_pos; y_pos; theta] after time Ts
%
% Dynamics:
%   dx/dt = v * cos(theta)
%   dy/dt = v * sin(theta)
%   dtheta/dt = omega
%
% Discretization: Euler integration
%
% See also: nlmpc, unicycleStateJacobian

% Author: GitHub Copilot + User
% Date: October 13, 2025
% Method: Method 5 (pureMPC) - True receding horizon NMPC

%% Extract state variables
x_pos = x(1);
y_pos = x(2);
theta = x(3);

%% Extract control inputs
v = u(1);
omega = u(2);

%% Compute next state using Euler integration
x_next = zeros(3, 1);
x_next(1) = x_pos + Ts * v * cos(theta);
x_next(2) = y_pos + Ts * v * sin(theta);
x_next(3) = theta + Ts * omega;

%% Normalize theta to [-pi, pi]
% This prevents angle wrapping issues during optimization
x_next(3) = atan2(sin(x_next(3)), cos(x_next(3)));

end
