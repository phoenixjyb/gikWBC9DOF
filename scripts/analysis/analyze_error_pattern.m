%% Analyze error pattern in staged run
clear; clc;

load('results/20251014_145719_fresh_no_discs/log_staged_ppForIk.mat');
s = log.stageLogs.stageC;
errors = s.positionErrorNorm * 1000; % mm

fprintf('========================================\n');
fprintf('  Error Pattern Analysis\n');
fprintf('========================================\n\n');

fprintf('First 10 errors (mm): ');
fprintf('%.1f ', errors(1:10));
fprintf('\n');

fprintf('Last 10 errors (mm): ');
fprintf('%.1f ', errors(end-9:end));
fprintf('\n\n');

% Split analysis
halfPoint = floor(length(errors) / 2);
firstHalf = errors(1:halfPoint);
secondHalf = errors(halfPoint+1:end);

fprintf('FIRST HALF (waypoints 1-%d):\n', halfPoint);
fprintf('  Mean error: %.1f mm\n', mean(firstHalf));
fprintf('  Failures (>500mm): %d/%d (%.1f%%)\n', ...
    sum(firstHalf > 500), length(firstHalf), ...
    sum(firstHalf > 500) / length(firstHalf) * 100);
fprintf('  Good (<5mm): %d/%d (%.1f%%)\n\n', ...
    sum(firstHalf < 5), length(firstHalf), ...
    sum(firstHalf < 5) / length(firstHalf) * 100);

fprintf('SECOND HALF (waypoints %d-%d):\n', halfPoint+1, length(errors));
fprintf('  Mean error: %.1f mm\n', mean(secondHalf));
fprintf('  Failures (>500mm): %d/%d (%.1f%%)\n', ...
    sum(secondHalf > 500), length(secondHalf), ...
    sum(secondHalf > 500) / length(secondHalf) * 100);
fprintf('  Good (<5mm): %d/%d (%.1f%%)\n\n', ...
    sum(secondHalf < 5), length(secondHalf), ...
    sum(secondHalf < 5) / length(secondHalf) * 100);

% Find alternating pattern
fprintf('ALTERNATING PATTERN CHECK:\n');
evenIdx = 2:2:length(errors);
oddIdx = 1:2:length(errors);

fprintf('  Even waypoints (2,4,6,...): Mean = %.1f mm, Failures = %d/%d\n', ...
    mean(errors(evenIdx)), sum(errors(evenIdx) > 500), length(evenIdx));
fprintf('  Odd waypoints (1,3,5,...):  Mean = %.1f mm, Failures = %d/%d\n\n', ...
    mean(errors(oddIdx)), sum(errors(oddIdx) > 500), length(oddIdx));

% Check for continuous blocks
fprintf('FAILURE BLOCKS:\n');
failMask = errors > 500;
transitions = diff([0; failMask; 0]);
starts = find(transitions == 1);
ends = find(transitions == -1) - 1;

if ~isempty(starts)
    for i = 1:min(5, length(starts))
        blockLen = ends(i) - starts(i) + 1;
        fprintf('  Block %d: waypoints %d-%d (%d consecutive failures)\n', ...
            i, starts(i), ends(i), blockLen);
    end
    if length(starts) > 5
        fprintf('  ... and %d more blocks\n', length(starts) - 5);
    end
else
    fprintf('  No continuous failure blocks found\n');
end
fprintf('\n');

% Visual pattern
fprintf('VISUAL PATTERN (first 50 waypoints):\n  ');
for i = 1:min(50, length(errors))
    if errors(i) < 5
        fprintf('.');
    elseif errors(i) < 50
        fprintf('o');
    elseif errors(i) < 500
        fprintf('O');
    else
        fprintf('X');
    end
    if mod(i, 10) == 0
        fprintf(' ');
    end
end
fprintf('\n  (. = good <5mm, o = ok <50mm, O = bad <500mm, X = fail >500mm)\n\n');

fprintf('VISUAL PATTERN (last 50 waypoints):\n  ');
startIdx = max(1, length(errors) - 49);
for i = startIdx:length(errors)
    if errors(i) < 5
        fprintf('.');
    elseif errors(i) < 50
        fprintf('o');
    elseif errors(i) < 500
        fprintf('O');
    else
        fprintf('X');
    end
    if mod(i - startIdx + 1, 10) == 0
        fprintf(' ');
    end
end
fprintf('\n  (. = good <5mm, o = ok <50mm, O = bad <500mm, X = fail >500mm)\n');
