function generateCode(outputDir)
%GENERATECODE Run MATLAB Coder to emit C++ for GIK primitives.
%   gik9dof.codegen.generateCode(outputDir) generates C++ source in the
%   specified directory (default ''codegen''). Ensure
%   gik9dof.codegen.generateRobotModelData has been executed beforehand to
%   cache the rigidBodyTree model.

if nargin < 1 || strlength(outputDir) == 0
    outputDir = "codegen";
end

outputDirChar = char(outputDir);
if isempty(regexp(outputDirChar, '^[A-Za-z]:', 'once')) && ~startsWith(outputDirChar, '\\')
    outputDirChar = char(fullfile(pwd, outputDirChar));
end
fprintf('Using output dir: %s\n', outputDirChar);
if ~exist(outputDirChar, 'dir')
    mkdir(outputDirChar);
end

packageDir = fileparts(mfilename('fullpath'));
stepEntry = fullfile(packageDir, 'solveGIKStep.m');
followEntry = fullfile(packageDir, 'followTrajectory.m');

cfg = coder.config('lib', 'ecoder', false);
cfg.TargetLang = 'C++';
cfg.TargetLangStandard = 'C++17 (ISO)';
cfg.GenCodeOnly = true;
cfg.GenerateReport = true;
cfg.HardwareImplementation.ProdHWDeviceType = 'ARM Compatible->ARM 64-bit (LP64)';
cfg.HardwareImplementation.TargetHWDeviceType = cfg.HardwareImplementation.ProdHWDeviceType;
cfg.HardwareImplementation.ProdBitPerInt = 32;
cfg.HardwareImplementation.ProdBitPerLong = 64;
cfg.HardwareImplementation.ProdBitPerLongLong = 64;
cfg.HardwareImplementation.ProdBitPerPointer = 64;
cfg.HardwareImplementation.ProdBitPerSizeT = 64;
cfg.HardwareImplementation.ProdLargestAtomicInteger = 'Char';
cfg.HardwareImplementation.ProdLargestAtomicFloat = 'Double';

argsStep = {coder.typeof(0, [9,1], [false,false]), coder.typeof(0, [4,4], [false,false]), coder.typeof(0), coder.typeof(0)};
codegen('-config', cfg, ...
    stepEntry, ...
    '-args', argsStep, ...
    '-o', fullfile(outputDirChar, 'solveGIKStep'), ...
    '-d', outputDirChar);

maxWaypoints = 50;
posesType = coder.typeof(0, [4,4,maxWaypoints], [false,false,true]);
argsFollow = {coder.typeof(0, [9,1], [false,false]), posesType, coder.typeof(0), coder.typeof(0)};
codegen('-config', cfg, ...
    followEntry, ...
    '-args', argsFollow, ...
    '-o', fullfile(outputDirChar, 'followTrajectory'), ...
    '-d', outputDirChar);
end


