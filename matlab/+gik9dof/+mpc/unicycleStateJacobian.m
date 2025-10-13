function [A, B] = unicycleStateJacobian(x, u, Ts)
% UNICYCLESTATEJAC Jacobian of whole-body state transition
%
% Computes the Jacobian matrices of the discrete-time whole-body dynamics
% for improved NMPC convergence speed. This is optional but recommended.
%
% Inputs:
%   x  - [9x1] Current state [x_pos; y_pos; theta; q_arm(6)]
%   u  - [8x1] Control inputs [v; omega; q_dot_arm(6)]
%   Ts - Sample time (s)
%
% Outputs:
%   A - [9x9] State Jacobian: ∂x_next/∂x
%   B - [9x8] Input Jacobian: ∂x_next/∂u
%
% Linearized dynamics: x_next ≈ A*x + B*u
%
% See also: nlmpc, unicycleStateFcn

% Author: GitHub Copilot + User
% Date: October 13, 2025
% Method: Method 5 (pureMPC) - Whole-body MPC

%% Extract variables
theta = x(3);
v = u(1);

%% State Jacobian: A = ∂x_next/∂x (9x9)
% Base states affect themselves:
%   x_next(1) = x(1) + Ts * v * cos(theta)  → ∂/∂theta = -Ts*v*sin(theta)
%   x_next(2) = x(2) + Ts * v * sin(theta)  → ∂/∂theta = Ts*v*cos(theta)
%   x_next(3) = x(3) + Ts * omega
% Arm states are independent:
%   x_next(4:9) = x(4:9) + Ts * q_dot_arm

A = eye(9);
A(1, 3) = -Ts * v * sin(theta);  % ∂x_next(1)/∂theta
A(2, 3) = Ts * v * cos(theta);   % ∂x_next(2)/∂theta
% All other partials are 1 (diagonal) or 0 (off-diagonal)

%% Input Jacobian: B = ∂x_next/∂u (9x8)
% Base inputs affect base states:
%   ∂x_next(1)/∂v = Ts*cos(theta), ∂x_next(1)/∂ω = 0
%   ∂x_next(2)/∂v = Ts*sin(theta), ∂x_next(2)/∂ω = 0
%   ∂x_next(3)/∂v = 0,             ∂x_next(3)/∂ω = Ts
% Arm inputs affect arm states:
%   ∂x_next(4:9)/∂q_dot_arm(1:6) = Ts * I(6x6)

B = zeros(9, 8);

% Base state derivatives w.r.t. [v, omega]
B(1, 1) = Ts * cos(theta);  % ∂x/∂v
B(2, 1) = Ts * sin(theta);  % ∂y/∂v
B(3, 2) = Ts;               % ∂θ/∂ω

% Arm state derivatives w.r.t. q_dot_arm
B(4:9, 3:8) = Ts * eye(6);  % ∂q_arm/∂q_dot_arm

end
