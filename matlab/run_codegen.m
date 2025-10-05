scriptDir = fileparts(mfilename('fullpath'));
projectRoot = fileparts(scriptDir);
addpath(genpath(projectRoot));
cd(scriptDir);
outputDir = fullfile(projectRoot, 'codegen', 'linux_arm64');
gik9dof.codegen.generateRobotModelData();
gik9dof.codegen.generateCode(outputDir);
