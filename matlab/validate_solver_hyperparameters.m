function report = validate_solver_hyperparameters()
%VALIDATE_SOLVER_HYPERPARAMETERS Compare solver parameters across MATLAB and C++ codegen
%   This script performs a sanity check to ensure that MATLAB-side solver
%   parameters match the C++ codegen wrapper parameters.
%
%   Purpose:
%     - Verify MaxIterations is consistent
%     - Verify MaxTime is consistent
%     - Verify AllowRandomRestart is consistent
%     - Verify GradientTolerance is consistent
%     - Verify SolutionTolerance is consistent
%
%   Usage:
%     report = validate_solver_hyperparameters();
%
%   Returns:
%     report - Structure containing comparison results and status

fprintf('=============================================================\n');
fprintf('GIK Solver Hyperparameter Sanity Check\n');
fprintf('=============================================================\n');
fprintf('Purpose: Verify MATLAB and C++ codegen use identical solver parameters\n\n');

%% Part 1: Extract parameters from C++ codegen wrapper
fprintf('[1/3] Reading C++ codegen wrapper parameters...\n');
wrapperFile = fullfile('matlab', '+gik9dof', '+codegen_inuse', 'solveGIKStepWrapper.m');

if ~isfile(wrapperFile)
    error('Cannot find wrapper file: %s', wrapperFile);
end

% Read the file
fileText = fileread(wrapperFile);

% Extract parameters using regex
cpp_MaxTime = extractParameter(fileText, 'MaxTime', '[\d\.]+');
cpp_MaxIterations = extractParameter(fileText, 'MaxIterations', '\d+');
cpp_AllowRandomRestart = extractParameter(fileText, 'AllowRandomRestart', '(true|false)');
cpp_SolutionTolerance = extractParameter(fileText, 'SolutionTolerance', '[\de\-\.]+');
cpp_GradientTolerance = extractParameter(fileText, 'GradientTolerance', '[\de\-\.]+');

fprintf('  C++ Codegen Wrapper Parameters:\n');
fprintf('    MaxTime:              %s\n', cpp_MaxTime);
fprintf('    MaxIterations:        %s\n', cpp_MaxIterations);
fprintf('    AllowRandomRestart:   %s\n', cpp_AllowRandomRestart);
fprintf('    SolutionTolerance:    %s\n', cpp_SolutionTolerance);
fprintf('    GradientTolerance:    %s\n', cpp_GradientTolerance);
fprintf('  ✓ Successfully extracted C++ parameters\n\n');

%% Part 2: Extract default parameters from createGikSolver
fprintf('[2/3] Reading MATLAB createGikSolver defaults...\n');
createSolverFile = fullfile('matlab', '+gik9dof', 'createGikSolver.m');

if ~isfile(createSolverFile)
    error('Cannot find createGikSolver file: %s', createSolverFile);
end

% Read the file
fileText = fileread(createSolverFile);

% Extract MaxIterations default from function arguments
% Look for pattern: options.MaxIterations ... = NUMBER
ml_MaxIterations_default = extractParameter(fileText, ...
    'options\.MaxIterations\s+\([^\)]+\)\s+[^\=]+=\s*', '\d+');

fprintf('  MATLAB createGikSolver Default:\n');
fprintf('    MaxIterations:        %s\n', ml_MaxIterations_default);
fprintf('  ✓ Successfully extracted MATLAB defaults\n\n');

%% Part 3: Create a test solver and verify parameters
fprintf('[3/3] Creating test MATLAB solver for runtime verification...\n');

% Create minimal robot model
robot = rigidBodyTree('DataFormat', 'column');
body1 = rigidBody('link1');
jnt1 = rigidBodyJoint('jnt1', 'revolute');
body1.Joint = jnt1;
addBody(robot, body1, 'base');

% Create solver with standard parameters
solver_test = generalizedInverseKinematics('RigidBodyTree', robot, ...
    'ConstraintInputs', {'pose'}, ...
    'SolverAlgorithm', 'LevenbergMarquardt');

% Extract actual runtime parameters (handle different field names)
params = solver_test.SolverParameters;
ml_MaxTime = params.MaxTime;
if isfield(params, 'MaxNumIteration')
    ml_MaxIterations = params.MaxNumIteration;
elseif isfield(params, 'MaxIterations')
    ml_MaxIterations = params.MaxIterations;
else
    ml_MaxIterations = NaN;
    warning('Could not find MaxIterations field');
end
ml_AllowRandomRestart = params.AllowRandomRestart;
ml_GradientTolerance = params.GradientTolerance;
ml_SolutionTolerance = params.SolutionTolerance;

fprintf('  MATLAB Runtime Solver Parameters (default):\n');
fprintf('    MaxTime:              %.4f\n', ml_MaxTime);
fprintf('    MaxIterations:        %d\n', ml_MaxIterations);
fprintf('    AllowRandomRestart:   %d\n', ml_AllowRandomRestart);
fprintf('    GradientTolerance:    %e\n', ml_GradientTolerance);
fprintf('    SolutionTolerance:    %e\n', ml_SolutionTolerance);
fprintf('  ✓ Successfully extracted runtime parameters\n\n');

%% Part 4: Compare and report
fprintf('=============================================================\n');
fprintf('COMPARISON RESULTS\n');
fprintf('=============================================================\n\n');

report.cpp_params = struct(...
    'MaxTime', str2double(cpp_MaxTime), ...
    'MaxIterations', str2double(cpp_MaxIterations), ...
    'AllowRandomRestart', strcmp(cpp_AllowRandomRestart, 'true'), ...
    'SolutionTolerance', str2double(cpp_SolutionTolerance), ...
    'GradientTolerance', str2double(cpp_GradientTolerance));

report.matlab_params = struct(...
    'MaxTime', ml_MaxTime, ...
    'MaxIterations', ml_MaxIterations, ...
    'AllowRandomRestart', ml_AllowRandomRestart, ...
    'GradientTolerance', ml_GradientTolerance, ...
    'SolutionTolerance', ml_SolutionTolerance);

report.matlab_default_MaxIterations = str2double(ml_MaxIterations_default);

% Check each parameter
allMatch = true;

fprintf('Parameter                   C++ Codegen        MATLAB Runtime     Status\n');
fprintf('-------------------------   ----------------   ----------------   --------\n');

% MaxTime - C++ has explicit value, MATLAB uses default
status = checkParam('MaxTime', report.cpp_params.MaxTime, report.matlab_params.MaxTime, 0.001);
fprintf('%-27s %-18.4f %-18.4f %s\n', 'MaxTime', ...
    report.cpp_params.MaxTime, report.matlab_params.MaxTime, status);
if ~strcmp(status, '✓ MATCH')
    allMatch = false;
end

% MaxIterations - Critical parameter!
status = checkParam('MaxIterations', report.cpp_params.MaxIterations, ...
    report.matlab_params.MaxIterations, 0);
fprintf('%-27s %-18d %-18d %s\n', 'MaxIterations', ...
    report.cpp_params.MaxIterations, report.matlab_params.MaxIterations, status);
if ~strcmp(status, '✓ MATCH')
    allMatch = false;
    fprintf('  ⚠️  WARNING: MaxIterations mismatch! This affects convergence!\n');
end

% AllowRandomRestart
status = checkParam('AllowRandomRestart', report.cpp_params.AllowRandomRestart, ...
    report.matlab_params.AllowRandomRestart, 0);
fprintf('%-27s %-18s %-18s %s\n', 'AllowRandomRestart', ...
    boolStr(report.cpp_params.AllowRandomRestart), ...
    boolStr(report.matlab_params.AllowRandomRestart), status);
if ~strcmp(status, '✓ MATCH')
    allMatch = false;
end

% GradientTolerance
status = checkParam('GradientTolerance', report.cpp_params.GradientTolerance, ...
    report.matlab_params.GradientTolerance, 1e-10);
fprintf('%-27s %-18.2e %-18.2e %s\n', 'GradientTolerance', ...
    report.cpp_params.GradientTolerance, report.matlab_params.GradientTolerance, status);
if ~strcmp(status, '✓ MATCH')
    allMatch = false;
end

% SolutionTolerance
status = checkParam('SolutionTolerance', report.cpp_params.SolutionTolerance, ...
    report.matlab_params.SolutionTolerance, 1e-10);
fprintf('%-27s %-18.2e %-18.2e %s\n', 'SolutionTolerance', ...
    report.cpp_params.SolutionTolerance, report.matlab_params.SolutionTolerance, status);
if ~strcmp(status, '✓ MATCH')
    allMatch = false;
end

fprintf('\n');
fprintf('=============================================================\n');

if allMatch
    fprintf('✅ ALL PARAMETERS MATCH!\n');
    fprintf('C++ codegen and MATLAB are using identical solver parameters.\n');
    report.status = 'PASS';
else
    fprintf('❌ PARAMETER MISMATCH DETECTED!\n');
    fprintf('C++ codegen and MATLAB are using DIFFERENT solver parameters.\n');
    fprintf('This may explain convergence differences between MATLAB and C++.\n');
    report.status = 'FAIL';
end

fprintf('=============================================================\n\n');

%% Part 5: Additional notes
fprintf('NOTES:\n');
fprintf('  • C++ codegen MaxIterations: %d (hardcoded in wrapper)\n', ...
    report.cpp_params.MaxIterations);
fprintf('  • MATLAB default MaxIterations: %d (from createGikSolver)\n', ...
    report.matlab_default_MaxIterations);
fprintf('  • Validation uses MATLAB default values, which may differ from\n');
fprintf('    runtime values if explicitly set via solver options.\n\n');

fprintf('RECOMMENDATIONS:\n');
if report.cpp_params.MaxIterations < 1000
    fprintf('  ⚠️  C++ MaxIterations is %d, which may be too low for complex IK problems\n', ...
        report.cpp_params.MaxIterations);
    fprintf('     Consider increasing to 1000 for better convergence.\n\n');
end

if ~report.cpp_params.AllowRandomRestart
    fprintf('  • AllowRandomRestart is disabled (good for real-time performance)\n\n');
end

report.timestamp = datetime('now');
fprintf('Report generated: %s\n', char(report.timestamp));
fprintf('=============================================================\n');

end

%% Helper functions
function value = extractParameter(text, paramName, valuePattern)
    % Extract parameter value from text using regex
    pattern = sprintf('%s\\s*=\\s*(%s)', paramName, valuePattern);
    tokens = regexp(text, pattern, 'tokens', 'once');
    if ~isempty(tokens)
        value = tokens{1};
    else
        value = 'NOT FOUND';
    end
end

function status = checkParam(name, cppVal, mlVal, tolerance)
    % Compare two parameter values
    if islogical(cppVal) || islogical(mlVal)
        if cppVal == mlVal
            status = '✓ MATCH';
        else
            status = '✗ DIFF';
        end
    else
        if abs(cppVal - mlVal) <= tolerance
            status = '✓ MATCH';
        else
            status = '✗ DIFF';
        end
    end
end

function str = boolStr(val)
    % Convert boolean to string
    if val
        str = 'true';
    else
        str = 'false';
    end
end
