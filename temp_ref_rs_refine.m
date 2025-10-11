--
function pathOut = rs_refine_path(mapData, resolution, pathIn, params)
%RS_REFINE_PATH Reeds–Shepp shortcutting with cusp penalty (codegen-ready)
% pathOut = rs_refine_path(mapData, resolution, pathIn, params)
% mapData : logical/double matrix (1=occupied), size MxN
% resolution : cells per meter (scalar)
% pathIn : Nx3 [x y theta] initial path
% params : struct with fields:
% Rmin (double) e.g., 4.0
% reverseCost (double) e.g., 3.0 (bias against reverse segments)
% inflationRadius (double) e.g., 0.3 (meters safety margin)
% validationDistance (double) e.g., 0.04 (meters discretization)
% step (double) e.g., 0.08 (sampling along RS seg)
% iters (int32) e.g., 300
% maxSpan (int32) e.g., 120 (max indices replaced)
% lambdaCusp (double) e.g., 1.5 (meters per cusp)
% seed (uint32) optional for reproducibility
%
% Code generation:
% %#codegen (uncomment the next line when running codegen)
%#codegen

% --- Build collision checker over an inflated occupancy map ---
map = binaryOccupancyMap(mapData, resolution);
if params.inflationRadius > 0
inflate(map, params.inflationRadius); % clearance baked into the map
end
ss = stateSpaceSE2; % only needed by validator
ss.StateBounds = [map.XWorldLimits; map.YWorldLimits; [-pi pi]];
sv = validatorOccupancyMap(ss, Map=map);
sv.ValidationDistance = params.validationDistance; % sampling for motion checks

% --- RS connector ---
conn = reedsSheppConnection('MinTurningRadius', params.Rmin);
conn.ReverseCost = params.reverseCost; % bias away from reverse moves

% Optional: disable awkward types if your platform dislikes certain families
% conn.DisabledPathTypes = {"LpRnLp"}; % example; usually not needed

% --- Main loop ---
if isfield(params,'seed'); rng(double(params.seed)); end
P = pathIn;
N = size(P,1);
for it = 1:params.iters
if N < 3, break; end
i = randi([1, max(1, N-2)]); % pick two indices
spanMax = int32(min(params.maxSpan, N - i));
if spanMax < 2, continue; end
j = i + randi([2, spanMax]); % ensure at least 1 interior point

% Best RS connection between P(i) and P(j)
segNew = bestRS(conn, P(i,:), P(j,:));
if isempty(segNew), continue; end

% Collision check the candidate segment (densified)
if ~isRSConnectionValid(sv, segNew, params.step), continue; end

% Compute cost of old subpath [i..j] and new segment
oldCost = 0.0;
for k = i:(j-1)
    segK = bestRS(conn, P(k,:), P(k+1,:));
    if isempty(segK) || ~isRSConnectionValid(sv, segK, params.step)
        oldCost = inf; break;   % conservative: if old piece is invalid, don't compare
    end
    oldCost = oldCost + segK.Length + params.lambdaCusp * countCusps(segK);
end
newCost = segNew.Length + params.lambdaCusp * countCusps(segNew);

% Accept if better
if newCost + 1e-9 < oldCost
    % Interpolate poses along new RS segment (include endpoints)
    posesNew = interpolateRS(segNew, params.step);
    % Splice: keep P(1..i), insert posesNew (without duplicating endpoints), keep P(j..end)
    P = [P(1:i, :);
         posesNew(2:end-1, :);
         P(j:end, :)];
    N = size(P,1);
end
end

pathOut = P;

end % rs_refine_path

% ---------- helpers ----------

function seg = bestRS(conn, a, b)
% Pick shortest RS connection from the set returned by connect()
seg = [];
try
[segs, costs] = connect(conn, a, b); % returns cell array and costs
if isempty(segs), return; end
[~, idx] = min(costs);
seg = segs{idx};
catch
seg = [];
end
end

function tf = isRSConnectionValid(sv, seg, step)
% Sample along the RS segment and check motion validity
L = seg.Length;
n = max(2, ceil(L/step)+1);
s = linspace(0, L, n);
poses = interpolate(seg, s); % [x y theta] at specified lengths
tf = true;
for k = 1:(size(poses,1)-1)
[ok, ~] = isMotionValid(sv, poses(k,:), poses(k+1,:));
if ~ok, tf = false; return; end
end
end

function poses = interpolateRS(seg, step)
L = seg.Length;
n = max(2, ceil(L/step)+1);
s = linspace(0, L, n);
poses = interpolate(seg, s);
end

function c = countCusps(seg)
% Count gear changes from MotionDirections (ignore trailing 'N' segments)
dirs = seg.MotionDirections(:).';
% Keep only the part corresponding to actual motions
types = seg.MotionTypes;
m = 0;
for t = 1:numel(types)
if types{t} ~= "N", m = m + 1; end
end
dirs = dirs(1:max(1,m));
c = sum(abs(diff(sign(dirs))) > 0);
end
