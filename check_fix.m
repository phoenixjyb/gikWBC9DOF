% Quick check of fix results
data = load('results/20251014_164148_fresh_no_discs/log_staged_ppForIk.mat');
logC = data.log.stageLogs.stageC;

% Calculate errors
posErr = logC.positionErrorNorm * 1000;
N = length(posErr);
firstHalf = posErr(1:floor(N/2));
secondHalf = posErr(floor(N/2)+1:end);

fprintf('Stage C Results:\n');
fprintf('  Overall mean: %.1f mm\n', mean(posErr));
fprintf('  First half:   %.1f mm\n', mean(firstHalf));
fprintf('  Second half:  %.1f mm\n', mean(secondHalf));
fprintf('  Max error:    %.1f mm\n', max(posErr));

if mean(secondHalf) > 100
    fprintf('\n❌ FIX DID NOT WORK - Second half still has high errors\n');
else
    fprintf('\n✅ FIX WORKED - Errors within acceptable range\n');
end
