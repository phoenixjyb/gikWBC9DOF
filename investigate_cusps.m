%% Deep Dive: Cusp and Path Quality Investigation
%  This script loads logs and visualizes Stage B paths to understand:
%  1. Where are the 2 cusps located?
%  2. Are cusps present in Hybrid A* output or introduced by RS?
%  3. What's the actual path smoothness profile?

clear; clc;
addpath(genpath('matlab'));

fprintf('=== Deep Dive: Cusp Investigation ===\n\n');

%% Load a representative log
fprintf('[1/4] Loading log from comprehensive test...\n');

logFile = 'results/test_comprehensive_evaluation.mat';
if ~isfile(logFile)
    error('Test results not found. Run test_comprehensive_evaluation first.');
end

data = load(logFile);
testResults = data.results;

% Use first config (Tuned Baseline)
log = testResults(1).log;
fprintf('  Analyzing: %s\n', testResults(1).name);

%% Extract Stage B path data
fprintf('\n[2/4] Extracting Stage B path data...\n');

if ~isfield(log, 'stageLogs') || ~isfield(log.stageLogs, 'stageB')
    error('No Stage B data in log');
end

stageB = log.stageLogs.stageB;
diag = stageB.diagnostics;

fprintf('  Cusp count: %d\n', diag.cuspCount);
fprintf('  Cusp locations (indices): %s\n', mat2str(diag.cuspLocations));
fprintf('  Path length: %d poses\n', size(stageB.pathStates, 1));

pathStates = stageB.pathStates;  % Nx3 [x, y, yaw]
execStates = stageB.execBaseStates;  % Executed path

%% Visualize cusps and curvature
fprintf('\n[3/4] Analyzing cusp characteristics...\n');

figure('Name', 'Stage B Path Analysis', 'Position', [100, 100, 1400, 900]);

% Subplot 1: Full path with cusp markers
subplot(2, 3, 1);
plot(pathStates(:, 1), pathStates(:, 2), 'b-', 'LineWidth', 2);
hold on;

% Mark cusps
if diag.cuspCount > 0
    cuspIdx = diag.cuspLocations;
    plot(pathStates(cuspIdx, 1), pathStates(cuspIdx, 2), 'ro', ...
         'MarkerSize', 15, 'LineWidth', 3);
    
    % Label cusps
    for i = 1:length(cuspIdx)
        text(pathStates(cuspIdx(i), 1), pathStates(cuspIdx(i), 2), ...
             sprintf('  C%d', i), 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'r');
    end
end

% Mark start and end
plot(pathStates(1, 1), pathStates(1, 2), 'gs', 'MarkerSize', 12, 'LineWidth', 2);
plot(pathStates(end, 1), pathStates(end, 2), 'ms', 'MarkerSize', 12, 'LineWidth', 2);

grid on; axis equal;
xlabel('X (m)'); ylabel('Y (m)');
title(sprintf('Stage B Path (Cusps: %d)', diag.cuspCount));
legend('Path', 'Cusps', 'Start', 'Goal', 'Location', 'best');

% Subplot 2: Curvature profile
subplot(2, 3, 2);
plot(diag.baseCurvature, 'b-', 'LineWidth', 1.5);
hold on;
yline(0, 'k--', 'LineWidth', 1);

% Mark cusp locations
if diag.cuspCount > 0
    plot(cuspIdx, diag.baseCurvature(cuspIdx), 'ro', ...
         'MarkerSize', 10, 'LineWidth', 2);
end

grid on;
xlabel('Path Index');
ylabel('Curvature (1/m)');
title(sprintf('Curvature Profile (max=%.2f, mean=%.2f)', ...
      diag.maxCurvature, diag.meanCurvature));
legend('Curvature', 'Zero', 'Cusps', 'Location', 'best');

% Subplot 3: Heading angle
subplot(2, 3, 3);
headingDeg = rad2deg(pathStates(:, 3));
plot(headingDeg, 'b-', 'LineWidth', 1.5);
hold on;

% Mark cusps
if diag.cuspCount > 0
    plot(cuspIdx, headingDeg(cuspIdx), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
end

grid on;
xlabel('Path Index');
ylabel('Heading (deg)');
title('Heading Angle Profile');
legend('Heading', 'Cusps', 'Location', 'best');

% Subplot 4: Velocity direction (forward vs reverse)
subplot(2, 3, 4);
% Compute displacement vectors
dx = diff(pathStates(:, 1));
dy = diff(pathStates(:, 2));
displacement = sqrt(dx.^2 + dy.^2);

% Check if moving forward or backward relative to heading
% Forward: movement aligns with heading
% Backward: movement opposes heading
movementAngle = atan2(dy, dx);
headingMidpoint = (pathStates(1:end-1, 3) + pathStates(2:end, 3)) / 2;
angleDiff = wrapToPi(movementAngle - headingMidpoint);

isForward = abs(angleDiff) < pi/2;
direction = ones(length(isForward), 1);
direction(~isForward) = -1;  % -1 for reverse

plot(direction, 'b-', 'LineWidth', 1.5);
yline(0, 'k--', 'LineWidth', 1);
hold on;

% Mark transitions (potential cusps)
transitions = find(diff(direction) ~= 0);
if ~isempty(transitions)
    plot(transitions, direction(transitions), 'mo', ...
         'MarkerSize', 10, 'LineWidth', 2);
end

% Mark reported cusps
if diag.cuspCount > 0
    % Adjust index for diff
    cuspIdxAdj = max(1, min(cuspIdx - 1, length(direction)));
    plot(cuspIdxAdj, direction(cuspIdxAdj), 'ro', ...
         'MarkerSize', 12, 'LineWidth', 3);
end

grid on;
xlabel('Path Index');
ylabel('Direction (+1=fwd, -1=rev)');
title(sprintf('Movement Direction (Transitions: %d)', length(transitions)));
if ~isempty(transitions) && diag.cuspCount > 0
    legend({'Direction', 'Zero', 'Transitions', 'Reported Cusps'}, 'Location', 'best');
elseif ~isempty(transitions)
    legend({'Direction', 'Zero', 'Transitions'}, 'Location', 'best');
elseif diag.cuspCount > 0
    legend({'Direction', 'Zero', 'Reported Cusps'}, 'Location', 'best');
else
    legend({'Direction', 'Zero'}, 'Location', 'best');
end
ylim([-1.5, 1.5]);

% Subplot 5: Cusp zoom-in (first cusp)
if diag.cuspCount > 0
    subplot(2, 3, 5);
    
    % Zoom window: ±10 poses around cusp
    idx1 = cuspIdx(1);
    windowSize = 10;
    idxRange = max(1, idx1-windowSize):min(size(pathStates, 1), idx1+windowSize);
    
    plot(pathStates(idxRange, 1), pathStates(idxRange, 2), 'b.-', ...
         'LineWidth', 2, 'MarkerSize', 15);
    hold on;
    plot(pathStates(idx1, 1), pathStates(idx1, 2), 'ro', ...
         'MarkerSize', 20, 'LineWidth', 3);
    
    % Draw heading arrows
    arrowLen = 0.05;
    for i = idxRange
        dx_arrow = arrowLen * cos(pathStates(i, 3));
        dy_arrow = arrowLen * sin(pathStates(i, 3));
        quiver(pathStates(i, 1), pathStates(i, 2), dx_arrow, dy_arrow, ...
               0, 'k', 'LineWidth', 1, 'MaxHeadSize', 2);
    end
    
    grid on; axis equal;
    xlabel('X (m)'); ylabel('Y (m)');
    title(sprintf('Cusp 1 Detail (Index %d)', idx1));
    
    % Subplot 6: Cusp angle analysis
    subplot(2, 3, 6);
    
    if diag.cuspCount >= 1
        cuspAngles = zeros(diag.cuspCount, 1);
        
        for i = 1:diag.cuspCount
            idx = cuspIdx(i);
            
            if idx > 1 && idx < size(pathStates, 1)
                % Angle between incoming and outgoing segments
                v_in = pathStates(idx, 1:2) - pathStates(idx-1, 1:2);
                v_out = pathStates(idx+1, 1:2) - pathStates(idx, 1:2);
                
                % Normalize
                v_in = v_in / (norm(v_in) + 1e-6);
                v_out = v_out / (norm(v_out) + 1e-6);
                
                % Compute angle
                cosAngle = dot(v_in, v_out);
                cosAngle = max(-1, min(1, cosAngle));
                angleDeg = rad2deg(acos(cosAngle));
                cuspAngles(i) = angleDeg;
            else
                cuspAngles(i) = NaN;
            end
        end
        
        bar(cuspAngles);
        xlabel('Cusp Number');
        ylabel('Turn Angle (deg)');
        title('Cusp Severity');
        ylim([0, 180]);
        grid on;
        
        fprintf('\n  Cusp Severity:\n');
        for i = 1:diag.cuspCount
            fprintf('    Cusp %d: %.1f° (index %d)\n', ...
                    i, cuspAngles(i), cuspIdx(i));
        end
    end
end

sgtitle('Stage B Path Quality Analysis', 'FontSize', 14, 'FontWeight', 'bold');

%% Check Hybrid A* vs RS refined path
fprintf('\n[4/4] Checking Hybrid A* vs RS refinement...\n');

if isfield(stageB, 'planner') && isfield(stageB.planner, 'hybridAStar')
    hybInfo = stageB.planner.hybridAStar;
    
    fprintf('  Hybrid A* Solution:\n');
    if isfield(hybInfo, 'solutionFound')
        fprintf('    Solution found: %s\n', mat2str(hybInfo.solutionFound));
    end
    if isfield(hybInfo, 'pathLength')
        fprintf('    Path length: %.2f m\n', hybInfo.pathLength);
    end
    if isfield(hybInfo, 'numPoses')
        fprintf('    Num poses: %d\n', hybInfo.numPoses);
    end
end

fprintf('\n  Reeds-Shepp Refinement:\n');
fprintf('    Iterations: %d\n', diag.rsIterations);
fprintf('    Improvements: %d\n', diag.rsImprovements);
fprintf('    Acceptance rate: %.1f%%\n', diag.rsAcceptanceRate * 100);
fprintf('    Path length improvement: %.3f m\n', diag.rsPathLengthImprovement);

if diag.rsImprovements == 0
    fprintf('\n  ⚠ WARNING: RS made zero improvements!\n');
    fprintf('    → Cusps are likely from Hybrid A* and RS couldn''t smooth them\n');
    fprintf('    → Try: Higher lambdaCusp, more iterations, or different RS params\n');
else
    fprintf('\n  ✓ RS made %d improvements\n', diag.rsImprovements);
end

fprintf('\n  Clothoid Smoothing:\n');
fprintf('    Applied: %s\n', mat2str(diag.clothoidApplied));
if diag.clothoidApplied
    fprintf('    Segments: %d\n', diag.clothoidSegments);
end

%% Summary
fprintf('\n=== INVESTIGATION SUMMARY ===\n');
fprintf('Cusp Count: %d\n', diag.cuspCount);
fprintf('Path Smoothness Score: %.3f\n', diag.pathSmoothness);
fprintf('Max Curvature: %.2f (1/m)\n', diag.maxCurvature);

if diag.cuspCount > 0
    fprintf('\nCusp Locations:\n');
    for i = 1:diag.cuspCount
        pct = 100 * cuspIdx(i) / size(pathStates, 1);
        fprintf('  Cusp %d: Index %d (%.1f%% along path)\n', i, cuspIdx(i), pct);
    end
end

if diag.rsImprovements == 0
    fprintf('\n🔴 ROOT CAUSE: RS refinement not effective\n');
    fprintf('   → Cusps persist from Hybrid A* output\n');
    fprintf('   → Consider: Disable cusps in Hybrid A* or increase RS power\n');
else
    fprintf('\n🟡 PARTIAL SUCCESS: RS made improvements but cusps remain\n');
    fprintf('   → May need higher iteration count or different lambda\n');
end

fprintf('\nSuggested next steps:\n');
fprintf('1. Compare Hybrid A* raw output vs final path (before/after RS)\n');
fprintf('2. Test with AllowReverse=false to force forward-only motion\n');
fprintf('3. Try extreme lambdaCusp values (0.1, 10.0) to see impact\n');
fprintf('4. Run with StageBUseReedsShepp=false to isolate Hybrid A*\n');
