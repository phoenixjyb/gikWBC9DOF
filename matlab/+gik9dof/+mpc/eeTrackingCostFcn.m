function J = eeTrackingCostFcn(X, U, e, data, robot, eeBodyName, weights)
% EETACKINGCOSTFCN Custom cost function for end-effector tracking via FK
%
% Computes the cost for NMPC based on end-effector position/orientation error
% using forward kinematics. This replaces simple output tracking with
% task-space tracking.
%
% Inputs:
%   X          - [nx x (p+1)] Predicted state trajectory
%                States: [x, y, theta, q_arm(6)]
%   U          - [nu x p] Predicted control trajectory
%                Controls: [v, omega, q_dot_arm(6)]
%   e          - Current tracking error (from nlmpcmove)
%   data       - Additional data structure with field:
%                .References - [nr x (p+1)] Reference trajectory
%                              For EE tracking: [p_ref(3); R_ref_vec(9)] per step
%   robot      - rigidBodyTree object for forward kinematics
%   eeBodyName - char, name of end-effector body
%   weights    - struct with fields:
%                .position: Weight for position error
%                .orientation: Weight for orientation error
%                .input: Weight for input effort [v, omega, q_dot_arm]
%                .rate: Weight for input rate (smoothness)
%
% Output:
%   J - Scalar cost value
%
% Cost structure:
%   J = Σ_k [w_p*||p_ee(k) - p_ref(k)||² + w_R*||R_ee(k) - R_ref(k)||²_F
%            + w_u*||u(k)||² + w_Δu*||Δu(k)||²] + terminal_cost
%
% See also: nlmpc, getTransform

% Author: GitHub Copilot + User
% Date: October 13, 2025
% Method: Method 5 (pureMPC) - Whole-body MPC

%% Extract dimensions
p = size(U, 2);  % Prediction horizon length

%% Initialize cost
J = 0;

%% Extract reference trajectory
% data.References should be [12 x (p+1)]:
%   Rows 1-3: p_ref (position)
%   Rows 4-12: R_ref vectorized (3x3 rotation matrix as 9x1)
ref = data.References;

%% Stage costs (over horizon)
for k = 1:p
    % Current state
    x_k = X(:, k);
    
    % Build full joint configuration [9x1]
    q_full = x_k;  % [x, y, theta, q_arm(6)]
    
    % Compute forward kinematics
    try
        T_ee = getTransform(robot, q_full, eeBodyName);
        p_ee = T_ee(1:3, 4);
        R_ee = T_ee(1:3, 1:3);
    catch
        % FK failed (out of bounds, etc.) - penalize heavily
        J = J + 1e6;
        continue;
    end
    
    % Extract reference
    p_ref = ref(1:3, k);
    R_ref = reshape(ref(4:12, k), [3, 3]);
    
    % Position error
    e_pos = p_ee - p_ref;
    J_pos = weights.position * (e_pos' * e_pos);
    
    % Orientation error (Frobenius norm of rotation difference)
    R_error = R_ee - R_ref;
    J_rot = weights.orientation * sum(R_error(:).^2);
    
    % Input effort
    u_k = U(:, k);
    J_input = weights.input * (u_k' * u_k);
    
    % Rate cost (smoothness)
    if k > 1
        u_prev = U(:, k-1);
        du_k = u_k - u_prev;
        J_rate = weights.rate * (du_k' * du_k);
    else
        J_rate = 0;
    end
    
    % Accumulate stage cost
    J = J + J_pos + J_rot + J_input + J_rate;
end

%% Terminal cost (strong penalty on final pose error)
x_N = X(:, p+1);
q_full_N = x_N;

try
    T_ee_N = getTransform(robot, q_full_N, eeBodyName);
    p_ee_N = T_ee_N(1:3, 4);
    R_ee_N = T_ee_N(1:3, 1:3);
    
    p_ref_N = ref(1:3, p+1);
    R_ref_N = reshape(ref(4:12, p+1), [3, 3]);
    
    e_pos_N = p_ee_N - p_ref_N;
    R_error_N = R_ee_N - R_ref_N;
    
    % Terminal cost (5x weight)
    J_terminal = 5.0 * weights.position * (e_pos_N' * e_pos_N) + ...
                 5.0 * weights.orientation * sum(R_error_N(:).^2);
    
    J = J + J_terminal;
catch
    J = J + 1e6;
end

end
