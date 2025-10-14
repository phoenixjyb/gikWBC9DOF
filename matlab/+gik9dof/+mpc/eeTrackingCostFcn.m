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

%% Slack variable penalty (to satisfy nlmpc requirement when using custom constraints)
% Small penalty on slack variable to avoid warning about unused slack
if ~isempty(e) && any(e ~= 0)
    J = J + 1e3 * sum(e.^2);  % Penalize constraint violations
end

%% DEBUG: Check if cost function is being called properly
% Remove this after debugging
persistent call_count;
if isempty(call_count)
    call_count = 0;
end
call_count = call_count + 1;
if call_count <= 3
    fprintf('  [DEBUG] eeTrackingCostFcn called: p=%d, weights.position=%.1f\n', p, weights.position);
end

%% Extract reference trajectory
% data.References is passed by nlmpc as [(p+1) x 12]:
%   Columns 1-3: p_ref (position)
%   Columns 4-12: R_ref vectorized (3x3 rotation matrix as 9x1)
% But we need to access it row-wise: ref(k, :) for timestep k
ref = data.References;  % [(p+1) x 12]

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
    
    % Extract reference for timestep k (row k of ref matrix)
    p_ref = ref(k, 1:3)';  % Column vector [3x1]
    R_ref = reshape(ref(k, 4:12), [3, 3]);  % Reshape to [3x3]
    
    % Position error
    e_pos = p_ee - p_ref;
    J_pos = weights.position * (e_pos' * e_pos);
    
    % Orientation error (SIMPLIFIED: use trace formula instead of Frobenius norm)
    % ||R_ee - R_ref||²_F = 2*(3 - trace(R_ee' * R_ref))
    % This is faster to compute and numerically better conditioned
    trace_RRT = trace(R_ee' * R_ref);
    J_rot = weights.orientation * 2.0 * (3.0 - trace_RRT);
    
    % Input effort (u = [v, omega, q_dot_arm(6)])
    u_k = U(:, k);
    % Separate weights for base and arm inputs
    J_input_v = weights.input_v * u_k(1)^2;
    J_input_omega = weights.input_omega * u_k(2)^2;
    J_input_arm = weights.input_arm * sum(u_k(3:8).^2);
    J_input = J_input_v + J_input_omega + J_input_arm;
    
    % Rate cost (smoothness)
    if k > 1
        u_prev = U(:, k-1);
        du_k = u_k - u_prev;
        % Separate rate weights for base and arm
        J_rate_v = weights.rate_v * du_k(1)^2;
        J_rate_omega = weights.rate_omega * du_k(2)^2;
        J_rate_arm = weights.rate_arm * sum(du_k(3:8).^2);
        J_rate = J_rate_v + J_rate_omega + J_rate_arm;
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
    
    p_ref_N = ref(p+1, 1:3)';  % Terminal reference position [3x1]
    R_ref_N = reshape(ref(p+1, 4:12), [3, 3]);  % Terminal reference orientation [3x3]
    
    e_pos_N = p_ee_N - p_ref_N;
    
    % Terminal cost (use terminal weight multiplier from config)
    % Use simplified orientation error (trace formula)
    trace_RRT_N = trace(R_ee_N' * R_ref_N);
    J_terminal = weights.terminal * weights.position * (e_pos_N' * e_pos_N) + ...
                 weights.terminal * weights.orientation * 2.0 * (3.0 - trace_RRT_N);
    
    J = J + J_terminal;
catch
    J = J + 1e6;
end

%% DEBUG: Print cost breakdown for first few calls  
if call_count <= 3
    fprintf('  [DEBUG] Final cost J=%.2e\n', J);
end

end
