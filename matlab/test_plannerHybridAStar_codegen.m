%% Test if plannerHybridAStar supports code generation
% This script will attempt to check code generation support

% Try to inspect the class
mc = ?plannerHybridAStar;
disp('plannerHybridAStar class info:');
disp(mc);

% Check if it's a value or handle class
if mc.HandleCompatible
    disp('This is a handle class (typically NOT code-generable)');
else
    disp('This is a value class (might be code-generable)');
end

% Try to find code generation attributes
if ~isempty(mc.PropertyList)
    disp('Properties:');
    for i = 1:length(mc.PropertyList)
        fprintf('  %s\n', mc.PropertyList(i).Name);
    end
end

% Check methods
if ~isempty(mc.MethodList)
    disp('Methods:');
    for i = 1:min(10, length(mc.MethodList))
        fprintf('  %s\n', mc.MethodList(i).Name);
    end
end

% Try a simple code generation test
fprintf('\n=== Testing Code Generation ===\n');
try
    % Create a simple test function
    testFile = 'test_hybridAStar_codegen.m';
    fid = fopen(testFile, 'w');
    fprintf(fid, 'function path = test_hybridAStar_codegen()\n');
    fprintf(fid, '%%#codegen\n');
    fprintf(fid, 'map = occupancyMap(10, 10, 10);\n');
    fprintf(fid, 'ss = stateSpaceSE2;\n');
    fprintf(fid, 'validator = validatorOccupancyMap(ss);\n');
    fprintf(fid, 'validator.Map = map;\n');
    fprintf(fid, 'planner = plannerHybridAStar(validator);\n');
    fprintf(fid, 'path = [0 0 0];\n');
    fprintf(fid, 'end\n');
    fclose(fid);
    
    % Try to generate code
    cfg = coder.config('lib');
    codegen('-config', cfg, testFile);
    
    fprintf('SUCCESS: Code generation worked!\n');
    delete(testFile);
catch ME
    fprintf('FAILED: %s\n', ME.message);
    if exist(testFile, 'file')
        delete(testFile);
    end
end
