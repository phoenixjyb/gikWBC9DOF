%% Add debug output to verify Phase 2A parameter is being used
% This modifies baseSeedFromEE temporarily to print diagnostics

clear; clc;

fprintf('Creating instrumented version of baseSeedFromEE...\n\n');

% Read the original file
originalFile = 'matlab/+gik9dof/baseSeedFromEE.m';
backupFile = 'matlab/+gik9dof/baseSeedFromEE.m.backup';
fileContent = fileread(originalFile);

% Create backup if it doesn't exist
if ~isfile(backupFile)
    copyfile(originalFile, backupFile);
    fprintf('✓ Created backup: %s\n', backupFile);
end

% Check if already instrumented
if contains(fileContent, 'DEBUG_PHASE2A_PARAMETER_CHECK')
    fprintf('⚠️  File already instrumented!\n');
    fprintf('   Run restore_baseseedfromee.m to restore original\n\n');
    return;
end

% Find the line after "if options.Verbose"
insertAfter = 'if options.Verbose';
debugCode = sprintf(['\n' ...
    '    fprintf(''Generating base seed path from %%d EE waypoints...\\n'', nWaypoints);\n' ...
    '    fprintf(''DEBUG_PHASE2A_PARAMETER_CHECK: UseOrientationZNominal = %%d\\n'', options.UseOrientationZNominal); %% INSTRUMENTATION\n' ...
    '    fprintf(''DEBUG_PHASE2A_PARAMETER_CHECK: OrientationWeight = %%.2f\\n'', options.OrientationWeight); %% INSTRUMENTATION\n']);

% Replace the fprintf block
oldBlock = [insertAfter sprintf('\n    fprintf(''Generating base seed path from %%d EE waypoints...\\n'', nWaypoints);')];
newBlock = [insertAfter debugCode];

newContent = strrep(fileContent, oldBlock, newBlock);

% Write the modified file
fid = fopen(originalFile, 'w');
fprintf(fid, '%s', newContent);
fclose(fid);

fprintf('✓ Instrumented baseSeedFromEE.m with debug output\n\n');
fprintf('Now run:\n');
fprintf('  matlab -batch "debug_parameter_passing"\n\n');
fprintf('To restore original:\n');
fprintf('  mv matlab/+gik9dof/baseSeedFromEE.m.backup matlab/+gik9dof/baseSeedFromEE.m\n');
