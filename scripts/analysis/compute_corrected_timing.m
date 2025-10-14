% Quick recompute with corrected timing
load('results/20251013_183955_method_comparison_aggressive/log_method4_ppFirst_aggressive.mat');
load('results/20251013_151157_method_comparison/log_method1_ppForIk.mat');

logC1 = log1.stageLogs.stageC;
logC4 = log4.stageLogs.stageC;

fprintf('═══════════════════════════════════════════════════════════\n');
fprintf('  CORRECTED TIMING COMPARISON\n');
fprintf('═══════════════════════════════════════════════════════════\n\n');

stageCTime1 = logC1.time(end);
stageCTime4 = logC4.time(end);

fprintf('Method 1 (ppForIk) Stage C time: %.2f seconds (%.2f minutes)\n', stageCTime1, stageCTime1/60);
fprintf('Method 4 (ppFirst) Stage C time: %.2f seconds (%.2f minutes)\n', stageCTime4, stageCTime4/60);
fprintf('Speedup: %.1fx faster\n\n', stageCTime1/stageCTime4);

fprintf('Accuracy comparison:\n');
fprintf('  Method 1 EE error: %.2f mm (mean)\n', mean(logC1.positionErrorNorm)*1000);
fprintf('  Method 4 EE error: %.2f mm (mean)\n\n', mean(logC4.positionErrorNorm)*1000);

fprintf('Method 4 Performance:\n');
fprintf('  Fallback rate: %.1f%%\n', logC4.diagnostics.ppFirst.fallbackCount/logC4.diagnostics.ppFirst.totalWaypoints*100);
fprintf('  Convergence rate: %.1f%%\n', sum(logC4.successMask)/length(logC4.successMask)*100);
fprintf('  Mean GIK iterations: %.1f\n\n', mean(logC4.iterations));

fprintf('═══════════════════════════════════════════════════════════\n');
fprintf('CONCLUSION:\n');
fprintf('═══════════════════════════════════════════════════════════\n\n');

if stageCTime4 < stageCTime1 * 0.5 && logC4.diagnostics.ppFirst.fallbackCount/logC4.diagnostics.ppFirst.totalWaypoints < 0.2
    fprintf('✓ Method 4 is SIGNIFICANTLY FASTER with acceptable accuracy\n');
elseif logC4.diagnostics.ppFirst.fallbackCount/logC4.diagnostics.ppFirst.totalWaypoints > 0.4
    fprintf('✗ Method 4 fallback rate too high (>40%%) - needs tuning\n');
elseif mean(logC4.positionErrorNorm)*1000 > mean(logC1.positionErrorNorm)*1000 * 2
    fprintf('✗ Method 4 accuracy too poor (>2x Method 1 error)\n');
else
    fprintf('⚠ Method 4 shows promise but needs further optimization\n');
end
