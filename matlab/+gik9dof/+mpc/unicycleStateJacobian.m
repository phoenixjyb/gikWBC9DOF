function [A, B] = unicycleStateJacobian(x, u, Ts)
% UNICYCLESTATEJAC Jacobian of unicycle state transition
%
% Computes the Jacobian matrices of the discrete-time unicycle dynamics
% for improved NMPC convergence speed. This is optional but recommended.
%
% Inputs:
%   x  - [3x1] Current state [x_pos; y_pos; theta]
%   u  - [2x1] Control inputs [v; omega]
%   Ts - Sample time (s)
%
% Outputs:
%   A - [3x3] State Jacobian: ∂x_next/∂x
%   B - [3x2] Input Jacobian: ∂x_next/∂u
%
% Linearized dynamics: x_next ≈ A*x + B*u
%
% See also: nlmpc, unicycleStateFcn

% Author: GitHub Copilot + User
% Date: October 13, 2025
% Method: Method 5 (pureMPC)

%% Extract variables
theta = x(3);
v = u(1);

%% State Jacobian: A = ∂x_next/∂x
% x_next(1) = x(1) + Ts * v * cos(theta)
% x_next(2) = x(2) + Ts * v * sin(theta)
% x_next(3) = x(3) + Ts * omega

A = [1,  0,  -Ts * v * sin(theta);
     0,  1,   Ts * v * cos(theta);
     0,  0,   1];

%% Input Jacobian: B = ∂x_next/∂u
% ∂x_next/∂v: [Ts*cos(theta); Ts*sin(theta); 0]
% ∂x_next/∂ω: [0; 0; Ts]

B = [Ts * cos(theta),  0;
     Ts * sin(theta),  0;
     0,                Ts];

end
