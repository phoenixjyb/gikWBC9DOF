function animateHolisticWithHelper(logHolistic, varargin)
%ANIMATEHOLISTICWITHHELPER Render holistic run using staged helper visuals.
%
%   gik9dof.animateHolisticWithHelper(logHolistic, ...) mirrors the staged
%   animation styling (floor discs, dark ground, overlays) by repackaging the
%   holistic log into a minimal staged-compatible struct and forwarding the
%   call to gik9dof.animateStagedWithHelper.

if nargin < 1 || ~isstruct(logHolistic)
    error('animateHolisticWithHelper:InvalidInput', ...
        'First argument must be the holistic log struct.');
end

stagedLike = struct();
stagedLike.qTraj = logHolistic.qTraj;
if isfield(logHolistic, 'timestamps') && ~isempty(logHolistic.timestamps)
    stagedLike.timestamps = logHolistic.timestamps;
else
    stagedLike.timestamps = [];
end
if isfield(logHolistic, 'rateHz')
    stagedLike.rateHz = logHolistic.rateHz;
end
if isfield(logHolistic, 'eePositions')
    stagedLike.eePositions = logHolistic.eePositions;
end
if isfield(logHolistic, 'floorDiscs')
    stagedLike.floorDiscs = logHolistic.floorDiscs;
end
if isfield(logHolistic, 'distanceSpecs')
    stagedLike.distanceSpecs = logHolistic.distanceSpecs;
end

stageC = struct();
stageC.qTraj = logHolistic.qTraj;
if isfield(logHolistic, 'timestamps')
    stageC.timestamps = logHolistic.timestamps;
else
    stageC.timestamps = [];
end
stagedLike.stageLogs = struct('stageC', stageC);

if isfield(logHolistic, 'environment')
    stagedLike.environment = logHolistic.environment;
end
if isfield(logHolistic, 'targetPositions')
    stagedLike.targetPositions = logHolistic.targetPositions;
end

% Inject helper options defaults
hasHelperOptions = any(strcmpi(varargin(1:2:end), 'HelperOptions'));
if ~hasHelperOptions
    varargin(end+1:end+2) = {'HelperOptions', struct()};
end

gik9dof.animateStagedWithHelper(stagedLike, varargin{:});
end
