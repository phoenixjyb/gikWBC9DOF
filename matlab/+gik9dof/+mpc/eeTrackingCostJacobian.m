function [dJdX, dJdU, dJde] = eeTrackingCostJacobian(X, U, e, data, robot, eeBodyName, weights)
% EETRACKINGCOSTJACOBIAN Analytical Jacobian of the EE tracking cost.
%
% Provides state and input gradients for the custom cost used by the
% whole-body MPC controller. This replaces finite-difference gradients in
% nlmpc to improve convergence speed.
%
% Inputs/outputs follow the nlmpc CustomCostJacobianFcn signature. See
% eeTrackingCostFcn for the corresponding scalar cost implementation.

nx = size(X, 1);
nu = size(U, 1);
p = size(U, 2);

dJdX = zeros(nx, p + 1);
dJdU = zeros(nu, p);

if isempty(e)
    dJde = zeros(0, 1);
else
    dJde = 2e3 * e;  % Gradient of slack penalty
end

ref = data.References;

% Helper weights (provide defaults in case fields are missing)
w_pos = getWeight(weights, 'position', 0);
w_ori = getWeight(weights, 'orientation', 0);
w_input_v = getWeight(weights, 'input_v', 0);
w_input_omega = getWeight(weights, 'input_omega', 0);
w_input_arm = getWeight(weights, 'input_arm', 0);
w_rate_v = getWeight(weights, 'rate_v', 0);
w_rate_omega = getWeight(weights, 'rate_omega', 0);
w_rate_arm = getWeight(weights, 'rate_arm', 0);
w_terminal = getWeight(weights, 'terminal', 0);

inputWeightVec = [w_input_v; w_input_omega; repmat(w_input_arm, 6, 1)];
rateWeightVec = [w_rate_v; w_rate_omega; repmat(w_rate_arm, 6, 1)];

% Optional posture regularisation
[hasBaseReg, baseWeights, baseNominal] = parseBaseReg(weights);
[hasArmReg, armWeights, armNominal] = parseArmReg(weights);

for k = 1:p
    xk = X(:, k);
    uk = U(:, k);
    
    % Forward kinematics and Jacobians
    try
        T_ee = getTransform(robot, xk, eeBodyName);
    catch
        % Fall back to zero gradient if FK fails (cost function penalises heavily)
        continue;
    end
    p_ee = T_ee(1:3, 4);
    R_ee = T_ee(1:3, 1:3);
    
    p_ref = ref(k, 1:3)';
    R_ref = reshape(ref(k, 4:12), [3, 3]);
    
    [C, ~] = gik9dof.mpc.eeOutputJacobian(xk, uk, robot, eeBodyName);
    J_pos = C(1:3, :);
    J_rot = C(4:12, :);
    
    % Position tracking gradient
    e_pos = p_ee - p_ref;
    if w_pos ~= 0
        grad_pos = 2 * w_pos * (J_pos.' * e_pos);
    else
        grad_pos = zeros(nx, 1);
    end
    
    % Orientation tracking gradient
    if w_ori ~= 0
        R_diff = R_ee - R_ref;
        grad_ori = 2 * w_ori * (J_rot.' * R_diff(:));
    else
        grad_ori = zeros(nx, 1);
    end
    
    % Base / arm posture gradients
    grad_posture = zeros(nx, 1);
    if hasBaseReg
        baseDiff = xk(1:3) - baseNominal;
        grad_posture(1:3) = 2 * baseWeights .* baseDiff;
    end
    if hasArmReg
        armDiff = xk(4:9) - armNominal;
        grad_posture(4:9) = 2 * armWeights .* armDiff;
    end
    
    dJdX(:, k) = dJdX(:, k) + grad_pos + grad_ori + grad_posture;
    
    % Input effort gradient
    dJdU(:, k) = dJdU(:, k) + 2 * inputWeightVec .* uk;
    
    % Rate (smoothness) gradient
    if k > 1 && any(rateWeightVec)
        delta = uk - U(:, k-1);
        grad_rate = 2 * rateWeightVec .* delta;
        dJdU(:, k) = dJdU(:, k) + grad_rate;
        dJdU(:, k-1) = dJdU(:, k-1) - grad_rate;
    end
end

% Terminal cost gradient (pose tracking only)
xN = X(:, p+1);
try
    T_ee_N = getTransform(robot, xN, eeBodyName);
    p_ee_N = T_ee_N(1:3, 4);
    R_ee_N = T_ee_N(1:3, 1:3);
    
    p_ref_N = ref(p+1, 1:3)';
    R_ref_N = reshape(ref(p+1, 4:12), [3, 3]);
    
    [C_N, ~] = gik9dof.mpc.eeOutputJacobian(xN, zeros(nu, 1), robot, eeBodyName);
    J_pos_N = C_N(1:3, :);
    J_rot_N = C_N(4:12, :);
    
    grad_pos_N = zeros(nx, 1);
    grad_ori_N = zeros(nx, 1);
    
    if w_terminal ~= 0 && w_pos ~= 0
        e_pos_N = p_ee_N - p_ref_N;
        grad_pos_N = 2 * w_terminal * w_pos * (J_pos_N.' * e_pos_N);
    end
    
    if w_terminal ~= 0 && w_ori ~= 0
        R_diff_N = R_ee_N - R_ref_N;
        grad_ori_N = 2 * w_terminal * w_ori * (J_rot_N.' * R_diff_N(:));
    end
    
    dJdX(:, p+1) = dJdX(:, p+1) + grad_pos_N + grad_ori_N;
catch
    % Ignore terminal gradient if FK/Jacobian fails
end

end

function value = getWeight(weights, fieldName, defaultValue)
    if isfield(weights, fieldName)
        value = weights.(fieldName);
    else
        value = defaultValue;
    end
end

function [flag, w, nominal] = parseBaseReg(weights)
    if isfield(weights, 'base_posture') && any(weights.base_posture)
        w = reshape(weights.base_posture, [], 1);
        if numel(w) == 1
            w = repmat(w, 3, 1);
        elseif numel(w) < 3
            w = [w(:); zeros(3-numel(w), 1)];
        else
            w = w(1:3);
        end
        if isfield(weights, 'base_nominal')
            nominal = reshape(weights.base_nominal, [], 1);
            if numel(nominal) < 3
                nominal = [nominal(:); zeros(3-numel(nominal), 1)];
            else
                nominal = nominal(1:3);
            end
        else
            nominal = zeros(3, 1);
        end
        flag = true;
    else
        flag = false;
        w = zeros(3, 1);
        nominal = zeros(3, 1);
    end
end

function [flag, w, nominal] = parseArmReg(weights)
    if isfield(weights, 'arm_posture') && any(weights.arm_posture)
        w = reshape(weights.arm_posture, [], 1);
        if numel(w) == 1
            w = repmat(w, 6, 1);
        elseif numel(w) < 6
            w = [w(:); zeros(6-numel(w), 1)];
        else
            w = w(1:6);
        end
        if isfield(weights, 'arm_nominal')
            nominal = reshape(weights.arm_nominal, [], 1);
            if numel(nominal) < 6
                nominal = [nominal(:); zeros(6-numel(nominal), 1)];
            else
                nominal = nominal(1:6);
            end
        else
            nominal = zeros(6, 1);
        end
        flag = true;
    else
        flag = false;
        w = zeros(6, 1);
        nominal = zeros(6, 1);
    end
end
