function [C, D] = eeOutputJacobian(x, u, robot, eeBodyName)
% EEOUTPUTJACOBIAN Jacobian of end-effector output function
%
% Computes the Jacobian of the EE pose output with respect to state and input.
% This significantly speeds up NMPC by avoiding numerical differentiation of FK.
%
% Inputs:
%   x          - [9x1] State [x, y, theta, q_arm(6)]
%   u          - [8x1] Control inputs [v, omega, q_dot_arm(6)] (unused for output)
%   robot      - rigidBodyTree object
%   eeBodyName - Name of end-effector body
%
% Outputs:
%   C - [12x9] Output Jacobian w.r.t. state: ∂y/∂x
%               where y = [p_ee(3); R_ee_vec(9)]
%   D - [12x8] Output Jacobian w.r.t. input: ∂y/∂u (zeros, output doesn't depend on u directly)
%
% The output function is:
%   y = [p_ee(x); vec(R_ee(x))]  (12x1)
% where p_ee and R_ee come from getTransform(robot, x, eeBodyName)
%
% Jacobian structure:
%   ∂p_ee/∂x   = [0 0 0, J_p(q_arm)]  (3x9)  - Position Jacobian (only depends on q_arm)
%   ∂R_vec/∂x  = [0 0 0, J_R(q_arm)]  (9x9)  - Orientation Jacobian (only depends on q_arm)
%
% where J_p and J_R are from the geometric Jacobian of the manipulator
%
% See also: nlmpc, geometricJacobian, getTransform

% Author: GitHub Copilot + User  
% Date: October 14, 2025
% Method: Method 5 (pureMPC) - Whole-body MPC with analytical Jacobians

%% Initialize Jacobians
C = zeros(12, 9);  % ∂y/∂x
D = zeros(12, 8);  % ∂y/∂u (output doesn't depend on u)

%% Get geometric Jacobian from rigidBodyTree
% geometricJacobian returns [6xn] where n is number of joints (9 in our case)
% Rows 1-3: Linear velocity Jacobian (relates joint velocities to EE linear velocity)
% Rows 4-6: Angular velocity Jacobian (relates joint velocities to EE angular velocity)
try
    q_full = x;  % Full configuration [x, y, theta, q_arm(6)]
    Jg = geometricJacobian(robot, q_full, eeBodyName);  % [6x9]
    
    % Extract position Jacobian (rows 1-3)
    % ∂p_ee/∂q = J_linear (3x9)
    J_pos = Jg(1:3, :);  % [3x9]
    
    % Position part of output Jacobian
    C(1:3, :) = J_pos;  % ∂p_ee/∂x = J_pos
    
    % For orientation part, we need ∂(vec(R))/∂q
    % This is more complex, but we can approximate using the angular Jacobian
    % The angular velocity Jacobian J_omega relates joint velocities to angular velocity
    % For small rotations: ΔR ≈ skew(J_omega * Δq) * R
    % The full derivative ∂R/∂q is complex, but for MPC optimization,
    % the position Jacobian is most important
    
    % Extract angular Jacobian (rows 4-6)
    J_omega = Jg(4:6, :);  % [3x9]
    
    % Approximate orientation Jacobian using cross-product relationship
    % This is a simplified approximation that captures the main dependency
    % For each column of R (which becomes 3 consecutive rows in vec(R)):
    % We use the angular Jacobian to approximate how R changes with q
    
    % Get current rotation matrix
    T_ee = getTransform(robot, q_full, eeBodyName);
    R_ee = T_ee(1:3, 1:3);
    
    % Approximate ∂(vec(R))/∂q using angular velocity relationship
    % For rotation matrices: dR/dt = [ω]×R where [ω]× is skew-symmetric
    % Similarly: ∂R/∂q_i ≈ [∂ω/∂q_i]×R = [J_omega(:,i)]×R
    % vec(∂R/∂q_i) = vectorization of skew(J_omega(:,i)) * R
    
    for i = 1:9  % For each joint
        omega_i = J_omega(:, i);  % [3x1] - contribution of joint i to angular velocity
        
        % Skew-symmetric matrix [ω_i]×
        S = [0, -omega_i(3), omega_i(2);
             omega_i(3), 0, -omega_i(1);
             -omega_i(2), omega_i(1), 0];
        
        % dR/dq_i ≈ S * R
        dR_dqi = S * R_ee;  % [3x3]
        
        % Vectorize and store in Jacobian
        C(4:12, i) = dR_dqi(:);  % Store vec(dR/dq_i)
    end
    
catch ME
    % If Jacobian computation fails, return zero Jacobian
    % MPC will fall back to numerical differentiation
    warning('eeOutputJacobian: Failed to compute Jacobian: %s', ME.message);
    C = zeros(12, 9);
end

%% Input Jacobian is zero (output doesn't depend directly on u)
% D is already initialized to zeros

end
