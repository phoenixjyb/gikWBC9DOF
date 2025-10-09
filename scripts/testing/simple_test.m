%% Simple ARM64 Test
cd('matlab');
addpath(genpath('.'));

fprintf('Testing ARM64 codegen...\n');

% Define types inline
cfg = coder.config('lib');
cfg.TargetLang = 'C++';
cfg.CppNamespace = 'gik9dof_purepursuit';
cfg.GenCodeOnly = true;

fprintf('Running codegen...\n');

try
    codegen('-config', cfg, 'purePursuitVelocityController', '-d', 'codegen/test_simple', '-report');
    fprintf('SUCCESS!\n');
catch e
    fprintf('FAILED:\n%s\n', e.message);
    for i = 1:length(e.stack)
        fprintf('  %s (line %d)\n', e.stack(i).name, e.stack(i).line);
    end
end
