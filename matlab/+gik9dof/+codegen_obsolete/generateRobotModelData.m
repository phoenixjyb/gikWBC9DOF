function generateRobotModelData()
%GENERATEROBOTMODELDATA Save rigidBodyTree for MATLAB Coder builds.
%   gik9dof.codegen.generateRobotModelData() loads the project URDF, creates
%   the rigidBodyTree via gik9dof.createRobotModel, and saves it to a MAT file
%   used by code generation entry points. Run this once (or whenever the URDF
%   changes) before invoking codegen.

robot = gik9dof.createRobotModel('Validate', true);
modelPath = fullfile(fileparts(mfilename('fullpath')), 'robotModel.mat');
save(modelPath, 'robot');
fprintf('Saved robot model for codegen to %s\n', modelPath);
end
