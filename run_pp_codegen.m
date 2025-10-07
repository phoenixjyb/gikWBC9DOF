%% Standalone Pure Pursuit Codegen Script
% This script runs from the workspace root

fprintf('Pure Pursuit Code Generation - Standalone\n');
fprintf('==========================================\n\n');

% Navigate to matlab directory
cd('matlab');
fprintf('Changed to: %s\n\n', pwd);

% Add paths
addpath(genpath('.'));

% Run the actual codegen script
generate_code_purePursuit;
