function report = comprehensiveEvaluation(log, trajStruct, obstacleDiscs, options)
%COMPREHENSIVEEVALUATION Complete 6-criteria evaluation for chassis path quality.
%   report = comprehensiveEvaluation(log, trajStruct, obstacleDiscs, options)
%   evaluates a trajectory log against all 6 criteria:
%   
%   1. EE reference path smoothness
%   2. Base reference path smoothness  
%   3. EE tracking accuracy to JSON waypoints
%   4. Actual collision intrusion (not margin)
%   5. Cusp count and severity
%   6. Sideways movement violations
%
%   Inputs:
%       log           - Trajectory log from runStagedReference
%       trajStruct    - Original trajectory with JSON waypoints
%       obstacleDiscs - Mx3 array of [x, y, radius]
%       options       - Evaluation options (weights, thresholds)
%
%   Outputs:
%       report.criterion1_eeRefSmoothness
%       report.criterion2_baseRefSmoothness
%       report.criterion3_eeTracking
%       report.criterion4_collisions
%       report.criterion5_cusps
%       report.criterion6_sideways
%       report.overallScore
%       report.weightedScores
%       report.violations

arguments
    log struct
    trajStruct struct
    obstacleDiscs (:,3) double = []
    options.Weights (1,6) double = [0.15, 0.15, 0.25, 0.20, 0.15, 0.10]
    options.SafteyMargin (1,1) double = 0.10
    options.Dt (1,1) double = 0.1
    options.Verbose (1,1) logical = false
end

report = struct();

% Extract stage logs
if isfield(log, 'stageLogs')
    stageB = log.stageLogs.stageB;
    stageC = log.stageLogs.stageC;
else
    error('Log does not contain staged execution data.');
end

%% CRITERION 1: EE Reference Path Smoothness
% Evaluate Stage C reference EE path (from initial IK pass)
if isfield(stageC, 'referenceInitialIk') && isfield(stageC.referenceInitialIk, 'eePositions')
    eeRefPath = stageC.referenceInitialIk.eePositions;  % 3xN
    c1 = gik9dof.evaluatePathSmoothness(eeRefPath, options.Dt);
    
    % Score: penalize high jerk and kinks
    % Target: rmsJerk < 5 m/s³, kinkCount < 3
    c1_jerkScore = exp(-c1.rmsJerk / 5);
    c1_kinkScore = exp(-c1.kinkCount / 3);
    c1.score = 0.6 * c1_jerkScore + 0.4 * c1_kinkScore;
else
    c1 = struct('score', 0, 'rmsJerk', NaN, 'kinkCount', NaN);
end
report.criterion1_eeRefSmoothness = c1;

%% CRITERION 2: Base Reference Path Smoothness  
% Evaluate Stage C reference base path (from pure pursuit)
if isfield(stageC, 'referenceBaseStates')
    baseRefPath = stageC.referenceBaseStates';  % 3xN [x; y; yaw]
    c2 = gik9dof.evaluatePathSmoothness(baseRefPath(1:2, :), options.Dt);
    
    % Score: similar to criterion 1
    c2_jerkScore = exp(-c2.rmsJerk / 5);
    c2_kinkScore = exp(-c2.kinkCount / 3);
    c2.score = 0.6 * c2_jerkScore + 0.4 * c2_kinkScore;
else
    c2 = struct('score', 0, 'rmsJerk', NaN, 'kinkCount', NaN);
end
report.criterion2_baseRefSmoothness = c2;

%% CRITERION 3: EE Tracking Accuracy to JSON Waypoints
% Compare executed EE to desired JSON waypoints
if isfield(stageC, 'eePositions') && isfield(trajStruct, 'EndEffectorPositions')
    desiredEE = trajStruct.EndEffectorPositions;  % 3xN
    executedEE = stageC.eePositions;  % 3xN
    
    % Match sizes
    nMin = min(size(desiredEE, 2), size(executedEE, 2));
    errors = vecnorm(executedEE(:, 1:nMin) - desiredEE(:, 1:nMin), 2, 1)';
    
    c3.meanError = mean(errors);
    c3.maxError = max(errors);
    c3.rmsError = sqrt(mean(errors.^2));
    c3.errorProfile = errors;
    
    % Score: target mean < 0.05m, max < 0.15m
    c3_meanScore = exp(-c3.meanError / 0.05);
    c3_maxScore = exp(-c3.maxError / 0.15);
    c3.score = 0.6 * c3_meanScore + 0.4 * c3_maxScore;
else
    c3 = struct('score', 0, 'meanError', NaN, 'maxError', NaN);
end
report.criterion3_eeTracking = c3;

%% CRITERION 4: Actual Collision Intrusion
% Check both Stage B and Stage C base paths
allBasePoses = [];
if isfield(stageB, 'execBaseStates')
    allBasePoses = [allBasePoses; stageB.execBaseStates];
end
if isfield(stageC, 'execBaseStates')
    allBasePoses = [allBasePoses; stageC.execBaseStates];
end

if ~isempty(allBasePoses) && ~isempty(obstacleDiscs)
    c4 = gik9dof.evaluateCollisionIntrusion(allBasePoses, obstacleDiscs, options.SafteyMargin);
    
    % Score: heavily penalize actual intrusions, lightly penalize margin violations
    actualPenalty = c4.actualIntrusions * 0.5;  % Each intrusion costs 0.5
    marginPenalty = c4.marginIntrusions * 0.05;  % Each margin violation costs 0.05
    c4.score = max(0, 1.0 - actualPenalty - marginPenalty);
else
    c4 = struct('score', 1.0, 'actualIntrusions', 0, 'marginIntrusions', 0);
end
report.criterion4_collisions = c4;

%% CRITERION 5: Cusp Count and Severity
% Use Stage B diagnostics
if isfield(stageB, 'diagnostics')
    diag = stageB.diagnostics;
    c5.cuspCount = diag.cuspCount;
    c5.cuspLocations = diag.cuspLocations;
    
    % Compute cusp severity (angle at each cusp)
    if c5.cuspCount > 0 && isfield(stageB, 'pathStates')
        basePath = stageB.pathStates;
        c5.cuspSeverities = zeros(c5.cuspCount, 1);
        
        for i = 1:c5.cuspCount
            idx = c5.cuspLocations(i);
            if idx > 1 && idx < size(basePath, 1)
                % Angle change at cusp
                v1 = basePath(idx, 1:2) - basePath(idx-1, 1:2);
                v2 = basePath(idx+1, 1:2) - basePath(idx, 1:2);
                
                if norm(v1) > 1e-6 && norm(v2) > 1e-6
                    v1 = v1 / norm(v1);
                    v2 = v2 / norm(v2);
                    cosAngle = dot(v1, v2);
                    cosAngle = max(-1, min(1, cosAngle));
                    angleDeg = rad2deg(acos(cosAngle));
                    c5.cuspSeverities(i) = angleDeg;
                else
                    c5.cuspSeverities(i) = 0;
                end
            else
                c5.cuspSeverities(i) = 0;
            end
        end
        c5.maxCuspSeverity = max(c5.cuspSeverities);
        c5.meanCuspSeverity = mean(c5.cuspSeverities);
    else
        c5.cuspSeverities = [];
        c5.maxCuspSeverity = 0;
        c5.meanCuspSeverity = 0;
    end
    
    % Score: target 0 cusps, penalize severity
    cuspCountPenalty = c5.cuspCount * 0.2;  % Each cusp costs 0.2
    severityPenalty = (c5.maxCuspSeverity / 180) * 0.3;  % 180° cusp costs 0.3
    c5.score = max(0, 1.0 - cuspCountPenalty - severityPenalty);
else
    c5 = struct('score', 1.0, 'cuspCount', 0, 'maxCuspSeverity', 0);
end
report.criterion5_cusps = c5;

%% CRITERION 6: Sideways Movement Violations
% Check both Stage B and Stage C executed paths
if ~isempty(allBasePoses)
    c6 = gik9dof.evaluateChassisConstraints(allBasePoses, options.Dt, struct());
    
    % Score: penalize violation percentage and max misalignment
    violationPenalty = c6.sidewaysViolationPct / 100;  % 100% violations -> -1.0
    alignmentPenalty = (c6.maxAlignmentError / 45) * 0.5;  % 45° -> -0.5
    c6.score = max(0, 1.0 - violationPenalty - alignmentPenalty);
else
    c6 = struct('score', 1.0, 'sidewaysViolations', 0, 'maxAlignmentError', 0);
end
report.criterion6_sideways = c6;

%% Overall Score (weighted combination)
weights = options.Weights;
weights = weights / sum(weights);  % Normalize

scores = [c1.score, c2.score, c3.score, c4.score, c5.score, c6.score];
weightedScores = scores .* weights;
overallScore = sum(weightedScores);

report.overallScore = overallScore;
report.individualScores = scores;
report.weightedScores = weightedScores;
report.weights = weights;

%% Identify Violations (any criterion with score < 0.7)
violations = struct();
violations.eeRefSmoothness = (c1.score < 0.7);
violations.baseRefSmoothness = (c2.score < 0.7);
violations.eeTracking = (c3.score < 0.7);
violations.collisions = (c4.score < 0.7);
violations.cusps = (c5.score < 0.7);
violations.sideways = (c6.score < 0.7);
violations.count = sum(structfun(@double, violations));

report.violations = violations;

%% Verbose output
if options.Verbose
    fprintf('=== Comprehensive Evaluation ===\n');
    fprintf('[Criterion 1] EE Ref Smoothness:   %.3f (jerk=%.2f, kinks=%d)\n', c1.score, c1.rmsJerk, c1.kinkCount);
    fprintf('[Criterion 2] Base Ref Smoothness: %.3f (jerk=%.2f, kinks=%d)\n', c2.score, c2.rmsJerk, c2.kinkCount);
    fprintf('[Criterion 3] EE Tracking:         %.3f (mean=%.4fm, max=%.4fm)\n', c3.score, c3.meanError, c3.maxError);
    fprintf('[Criterion 4] Collisions:          %.3f (actual=%d, margin=%d)\n', c4.score, c4.actualIntrusions, c4.marginIntrusions);
    fprintf('[Criterion 5] Cusps:               %.3f (count=%d, max severity=%.1f°)\n', c5.score, c5.cuspCount, c5.maxCuspSeverity);
    fprintf('[Criterion 6] Sideways:            %.3f (violations=%.1f%%, max angle=%.1f°)\n', c6.score, c6.sidewaysViolationPct, c6.maxAlignmentError);
    fprintf('\n[Overall Score] %.3f\n', overallScore);
    fprintf('[Violations] %d criterion/criteria failed (score < 0.7)\n', violations.count);
end

end
