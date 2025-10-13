function nlobj = configureNMPCWeights(nlobj, weights)
% CONFIGURENMPCWEIGHTS Configure cost weights for NMPC controller
%
% Sets up the quadratic cost function for nonlinear MPC tracking:
%   J = Σ [||y - yref||²_Q + ||u||²_R + ||Δu||²_S] + ||y_N - yref_N||²_P
%
% Inputs:
%   nlobj   - nlmpc controller object
%   weights - struct with fields:
%             .tracking: Output tracking weight (position)
%             .input_v: Linear velocity input weight
%             .input_omega: Angular velocity input weight
%             .rate_v: Linear velocity rate weight (smoothness)
%             .rate_omega: Angular velocity rate weight (smoothness)
%
% Output:
%   nlobj - Updated nlmpc controller with configured weights
%
% Usage:
%   nlobj = nlmpc(3, 3, 2);
%   weights.tracking = 100;
%   weights.input_v = 1;
%   weights.input_omega = 10;
%   weights.rate_v = 2;
%   weights.rate_omega = 20;
%   nlobj = configureNMPCWeights(nlobj, weights);
%
% See also: nlmpc

% Author: GitHub Copilot + User
% Date: October 13, 2025
% Method: Method 5 (pureMPC)

%% Output tracking weights
% States/Outputs are [x, y, θ]
% Track [x, y] heavily, θ less important (will be corrected naturally)
nlobj.Weights.OutputVariables = [weights.tracking, ...      % x position
                                  weights.tracking, ...      % y position
                                  weights.tracking * 0.1];   % θ heading

%% Manipulated variable (input) weights
% Penalize control effort to prevent aggressive maneuvers
% [v, ω]
nlobj.Weights.ManipulatedVariables = [weights.input_v, ...
                                       weights.input_omega];

%% Rate of change weights (input smoothness)
% Penalize sudden changes in control inputs (jerk reduction)
% Higher weights = smoother trajectory
if isfield(weights, 'rate_v') && isfield(weights, 'rate_omega')
    nlobj.Weights.ManipulatedVariablesRate = [weights.rate_v, ...
                                              weights.rate_omega];
else
    % Default: 2x input weights for smoothness
    nlobj.Weights.ManipulatedVariablesRate = [weights.input_v * 2, ...
                                              weights.input_omega * 2];
end

%% Optional: Terminal cost via custom cost function
% Not implemented by default - terminal weight is handled by longer horizon
% If needed, can add:
% nlobj.Optimization.CustomCostFcn = @(X,U,e,data) terminalCost(X,U,e,data,weights);

end
