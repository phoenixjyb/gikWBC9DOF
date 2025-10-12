% Startup script for gikWBC9DOF project
% Automatically adds project paths when MATLAB starts in this directory

fprintf('📦 Setting up gikWBC9DOF project paths...\n');

% Add main matlab directory (packages are at top level)
matlabDir = fullfile(pwd, 'matlab');
if exist(matlabDir, 'dir')
    addpath(matlabDir);
    fprintf('   Added: %s\n', matlabDir);
    
    % Refresh path cache for package functions
    rehash toolboxcache;
    
    % Quick verification (file-based check for package functions)
    chassisFile = fullfile(matlabDir, '+gik9dof', '+control', 'simulateChassisExecution.m');
    trajFile = fullfile(matlabDir, '+gik9dof', 'trackReferenceTrajectory.m');
    
    if exist(chassisFile, 'file')
        fprintf('   ✓ simulateChassisExecution.m found\n');
    else
        fprintf('   ✗ simulateChassisExecution.m NOT found\n');
    end
    
    if exist(trajFile, 'file')
        fprintf('   ✓ trackReferenceTrajectory.m found\n');
    else
        fprintf('   ✗ trackReferenceTrajectory.m NOT found\n');
    end
else
    warning('matlab directory not found at: %s', matlabDir);
end

fprintf('✅ Project setup complete!\n\n');
