% run_planner_codegen_wsl.m
% WSL Linux Code Generation Script for Planner ARM64
% Generates ARM64 Linux binaries for Hybrid A* Planner
% Run from WSL: matlab -batch "cd('/mnt/c/Users/yanbo/wSpace/codegenGIKsample/Trial/gikWBC9DOF'); run_planner_codegen_wsl"

fprintf('Starting Planner ARM64 Linux code generation...\n');

% Add paths
addpath('matlab');
addpath('scripts/codegen');

% Run the planner codegen script
fprintf('Running generate_code_planner_arm64.m...\n');
run('scripts/codegen/generate_code_planner_arm64.m');

fprintf('✓ Planner codegen complete!\n');
