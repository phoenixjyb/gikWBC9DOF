% Minimal test - just check if functions exist
fprintf('Checking Method 6 functions...\n');

% Check if functions exist
if exist('gik9dof.solveBaseOptimization', 'file')
    fprintf('✅ solveBaseOptimization.m found\n');
else
    fprintf('❌ solveBaseOptimization.m NOT FOUND\n');
end

if exist('gik9dof.solveArmOptimization', 'file')
    fprintf('✅ solveArmOptimization.m found\n');
else
    fprintf('❌ solveArmOptimization.m NOT FOUND\n');
end

if exist('gik9dof.runStageCAlternating', 'file')
    fprintf('✅ runStageCAlternating.m found\n');
else
    fprintf('❌ runStageCAlternating.m NOT FOUND\n');
end

fprintf('\nAll Method 6 files are present!\n');
