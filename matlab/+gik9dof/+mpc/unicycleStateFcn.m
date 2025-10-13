function x_next = unicycleStateFcn(x, u, Ts)
% UNICYCLESTATEFCN State transition function for whole-body NMPC
%
% This function defines the discrete-time dynamics of a 9-DOF mobile manipulator:
% - 3 DOF differential drive base (unicycle model)
% - 6 DOF arm (direct integration of joint velocities)
%
% Inputs:
%   x  - [9x1] Current state [x_pos; y_pos; theta; q_arm(6)]
%        x_pos: X position in world frame (m)
%        y_pos: Y position in world frame (m)
%        theta: Heading angle (rad)
%        q_arm: Arm joint angles (rad)
%   u  - [8x1] Control inputs [v; omega; q_dot_arm(6)]
%        v: Linear velocity (m/s)
%        omega: Angular velocity (rad/s)
%        q_dot_arm: Arm joint velocities (rad/s)
%   Ts - Sample time (s)
%
% Output:
%   x_next - [9x1] Next state after time Ts
%
% Dynamics:
%   Base (unicycle):
%     dx/dt = v * cos(theta)
%     dy/dt = v * sin(theta)
%     dtheta/dt = omega
%   Arm (velocity integration):
%     dq_arm/dt = q_dot_arm
%
% Discretization: Euler integration
%
% See also: nlmpc, wholeBodyStateJacobian

% Author: GitHub Copilot + User
% Date: October 13, 2025
% Method: Method 5 (pureMPC) - True whole-body receding horizon NMPC

%% Extract state variables
x_pos = x(1);
y_pos = x(2);
theta = x(3);
q_arm = x(4:9);

%% Extract control inputs
v = u(1);
omega = u(2);
q_dot_arm = u(3:8);

%% Compute next state using Euler integration
x_next = zeros(9, 1);

% Base dynamics (unicycle model)
x_next(1) = x_pos + Ts * v * cos(theta);
x_next(2) = y_pos + Ts * v * sin(theta);
x_next(3) = theta + Ts * omega;

% Arm dynamics (velocity integration)
x_next(4:9) = q_arm + Ts * q_dot_arm;

%% Normalize theta to [-pi, pi]
% This prevents angle wrapping issues during optimization
x_next(3) = atan2(sin(x_next(3)), cos(x_next(3)));

end
