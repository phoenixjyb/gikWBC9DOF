function [pathOut, info] = rsClothoidRefineCodegen(pathIn, params)
%RSCLOTHOIDREFINECODEGEN Codegen-compatible wrapper for rsClothoidRefine
%   This wrapper provides the same interface as rsClothoidRefine but without
%   the arguments validation block that prevents code generation.
%
%   Input:
%       pathIn - Nx3 array [x y yaw]
%       params - struct with fields:
%           .discretizationDistance (scalar double, default 0.05)
%           .maxNumWaypoints (scalar double, default 0)
%
%   Output:
%       pathOut - Mx3 array [x y yaw] smoothed path
%       info - struct with fitting diagnostics:
%           .discretizationDistance
%           .maxNumWaypoints
%           .totalArcLength
%           .maxAbsCurvature
%           .numSegments
%           .numForwardWaypoints
%           .numReverseWaypoints
%           .success (logical)
%
%   See also rsClothoidRefine

%#codegen

% Validate inputs
coder.inline('never');
assert(isreal(pathIn) && size(pathIn, 2) == 3, ...
    'pathIn must be Nx3 real array [x y yaw]');
assert(isstruct(params), 'params must be a struct');

% Call the actual implementation
% The original function will handle the arguments validation internally
[pathOut, info] = gik9dof.control.rsClothoidRefine(pathIn, params);

end
