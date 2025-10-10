function result = simulateChassisControllerCodegen(pathStates, opts)
%SIMULATECHASSISCONTROLLERCODEGEN Codegen-compatible wrapper for simulateChassisController
%   This wrapper converts a simple struct input into the name-value pair
%   format required by simulateChassisController, which cannot be directly
%   code-generated due to its arguments validation block.
%
%   Input:
%       pathStates - Nx3 array [x y yaw]
%       opts - struct with fields:
%           .SampleTime (scalar double, default 0.1)
%           .ControllerMode (scalar double 0/1/2, default 2)
%           .FollowerOptions (struct, default empty struct)
%
%   Output:
%       result - struct with fields:
%           .poses (Nx3 [x y yaw])
%           .commands (Nx2 [v omega])
%           .wheelSpeeds (Nx2 [wL wR])
%           .status (Nx1 integer)
%           .follower (struct or [])
%           .controllerMode (scalar)
%
%   See also simulateChassisController

%#codegen

% Validate inputs
coder.inline('never');
assert(isreal(pathStates) && size(pathStates, 2) == 3, ...
    'pathStates must be Nx3 real array');
assert(isstruct(opts), 'opts must be a struct');

% Extract options with defaults
if isfield(opts, 'SampleTime')
    sampleTime = opts.SampleTime;
    assert(isscalar(sampleTime) && sampleTime > 0, 'SampleTime must be positive scalar');
else
    sampleTime = 0.1;
end

if isfield(opts, 'ControllerMode')
    controllerMode = opts.ControllerMode;
    assert(isscalar(controllerMode) && ismember(controllerMode, [0 1 2]), ...
        'ControllerMode must be 0, 1, or 2');
else
    controllerMode = 2;
end

if isfield(opts, 'FollowerOptions')
    followerOptions = opts.FollowerOptions;
    assert(isstruct(followerOptions), 'FollowerOptions must be a struct');
else
    followerOptions = struct();
end

% Call the actual implementation with name-value pairs
result = gik9dof.control.simulateChassisController(pathStates, ...
    'SampleTime', sampleTime, ...
    'ControllerMode', controllerMode, ...
    'FollowerOptions', followerOptions);

end
