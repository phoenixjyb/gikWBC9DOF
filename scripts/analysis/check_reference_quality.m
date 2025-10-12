%% Quick Reference Path Quality Check
%  Extracts reference paths from existing logs and checks for jerkiness

clear; clc;
addpath(genpath('matlab'));

fprintf('=== Reference Path Quality Check ===\n\n');

% Load test results
logFile = 'results/test_comprehensive_evaluation.mat';
if ~isfile(logFile)
    error('Test results not found');
end

data = load(logFile);
log = data.results(1).log;

%% Check what reference data is available
fprintf('[1/3] Checking available reference data...\n');

stageC = log.stageLogs.stageC;
fieldList = fieldnames(stageC);
fprintf('  StageC fields: ');
for i = 1:length(fieldList)
    fprintf('%s', fieldList{i});
    if i < length(fieldList), fprintf(', '); end
end
fprintf('\n\n');

hasRefBase = isfield(stageC, 'referenceBaseStates');
hasRefEE = isfield(stageC, 'referenceInitialIk');
hasExecEE = isfield(stageC, 'eePositions');

fprintf('  Reference base states: %s\n', mat2str(hasRefBase));
fprintf('  Reference EE (initial IK): %s\n', mat2str(hasRefEE));
fprintf('  Executed EE positions: %s\n', mat2str(hasExecEE));

%% Analyze what we have
if hasRefBase
    refBase = stageC.referenceBaseStates;  % Nx3 [x, y, yaw]
    fprintf('\n[2/3] Reference Base Path Analysis:\n');
    fprintf('  Path length: %d poses\n', size(refBase, 1));
    
    % Compute velocity (approximate)
    dt = 0.1;  % 10Hz default
    dx = diff(refBase(:, 1)) / dt;
    dy = diff(refBase(:, 2)) / dt;
    velocity = sqrt(dx.^2 + dy.^2);
    
    % Compute acceleration
    accel = diff(velocity) / dt;
    
    % Compute jerk
    jerk = diff(accel) / dt;
    
    fprintf('  Velocity: mean=%.3f m/s, max=%.3f m/s\n', mean(velocity), max(velocity));
    fprintf('  Acceleration: RMS=%.3f m/s², max=%.3f m/s²\n', rms(accel), max(abs(accel)));
    fprintf('  Jerk: RMS=%.3f m/s³, max=%.3f m/s³\n', rms(jerk), max(abs(jerk)));
    
    % Check for kinks (sharp angle changes in velocity)
    if length(velocity) > 1
        velAngles = atan2(dy, dx);
        angleDiffs = diff(velAngles);
        angleDiffs = wrapToPi(angleDiffs);
        largeAngleChanges = abs(angleDiffs) > deg2rad(30);
        
        fprintf('  Velocity direction changes >30°: %d\n', sum(largeAngleChanges));
        if any(largeAngleChanges)
            kinkIdx = find(largeAngleChanges);
            fprintf('    At indices: %s\n', mat2str(kinkIdx(1:min(5, length(kinkIdx)))));
        end
    end
else
    fprintf('\n[2/3] ⚠ No reference base path found in log\n');
end

%% Check executed EE for comparison
if hasExecEE
    fprintf('\n[3/3] Executed EE Path Analysis:\n');
    
    execEE = stageC.eePositions;  % 3xN
    fprintf('  Path length: %d poses\n', size(execEE, 2));
    
    % EE velocity
    dt = 0.1;
    dx_ee = diff(execEE(1, :)) / dt;
    dy_ee = diff(execEE(2, :)) / dt;
    dz_ee = diff(execEE(3, :)) / dt;
    vel_ee = sqrt(dx_ee.^2 + dy_ee.^2 + dz_ee.^2);
    
    % EE acceleration
    accel_ee = diff(vel_ee) / dt;
    
    % EE jerk
    jerk_ee = diff(accel_ee) / dt;
    
    fprintf('  Velocity: mean=%.3f m/s, max=%.3f m/s\n', mean(vel_ee), max(vel_ee));
    fprintf('  Acceleration: RMS=%.3f m/s², max=%.3f m/s²\n', rms(accel_ee), max(abs(accel_ee)));
    fprintf('  Jerk: RMS=%.3f m/s³, max=%.3f m/s³\n', rms(jerk_ee), max(abs(jerk_ee)));
    
    % Kink detection in XY plane
    xyVel = [dx_ee; dy_ee];
    if size(xyVel, 2) > 1
        for i = 1:size(xyVel, 2)-1
            v1 = xyVel(:, i);
            v2 = xyVel(:, i+1);
            if norm(v1) > 1e-6 && norm(v2) > 1e-6
                angleDiff_ee(i) = acos(dot(v1, v2) / (norm(v1) * norm(v2)));
            else
                angleDiff_ee(i) = 0;
            end
        end
        largeKinks_ee = abs(angleDiff_ee) > deg2rad(30);
        fprintf('  Velocity direction changes >30°: %d\n', sum(largeKinks_ee));
    end
else
    fprintf('\n[3/3] ⚠ No executed EE path found\n');
end

%% Summary and Recommendations
fprintf('\n=== SUMMARY ===\n');

if hasRefBase
    if rms(jerk) > 10
        fprintf('🔴 Reference base path has HIGH jerk (%.1f m/s³)\n', rms(jerk));
        fprintf('   → This is the "kinky" observation source!\n');
        fprintf('   → Improve Stage C pure pursuit or initial IK\n');
    elseif rms(jerk) > 5
        fprintf('🟡 Reference base path has MODERATE jerk (%.1f m/s³)\n', rms(jerk));
        fprintf('   → Some kinkiness present, could be smoothed\n');
    else
        fprintf('✅ Reference base path is SMOOTH (jerk %.1f m/s³)\n', rms(jerk));
    end
end

if hasExecEE
    if rms(jerk_ee) > 10
        fprintf('🔴 Executed EE path has HIGH jerk (%.1f m/s³)\n', rms(jerk_ee));
        fprintf('   → Tracking introduces kinkiness\n');
    elseif rms(jerk_ee) > 5
        fprintf('🟡 Executed EE path has MODERATE jerk (%.1f m/s³)\n', rms(jerk_ee));
    else
        fprintf('✅ Executed EE path is SMOOTH (jerk %.1f m/s³)\n', rms(jerk_ee));
    end
end

fprintf('\nRecommendation: ');
if hasRefBase && rms(jerk) > 5
    fprintf('Focus on Stage C reference generation (pure pursuit + IK)\n');
elseif hasExecEE && rms(jerk_ee) > 5
    fprintf('Focus on Stage C tracking controller tuning\n');
else
    fprintf('Both reference and execution are smooth. Check animation rendering.\n');
end
