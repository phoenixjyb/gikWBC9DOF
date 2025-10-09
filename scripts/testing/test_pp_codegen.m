%% Test Pure Pursuit Code Generation
% This script tests the codegen with better error reporting

try
    % Change to matlab directory
    cd('matlab');
    
    % Run the codegen script
    run('generate_code_purePursuit.m');
    
    fprintf('\n\n===========================================\n');
    fprintf('SUCCESS: Code generation completed!\n');
    fprintf('===========================================\n');
    
catch ME
    fprintf('\n\n===========================================\n');
    fprintf('ERROR in code generation:\n');
    fprintf('===========================================\n');
    fprintf('Message: %s\n', ME.message);
    fprintf('\nStack trace:\n');
    for i = 1:length(ME.stack)
        fprintf('  %s (line %d)\n', ME.stack(i).name, ME.stack(i).line);
    end
    fprintf('===========================================\n');
    
    % Show full error report
    fprintf('\nFull error report:\n');
    fprintf('%s\n', getReport(ME));
    
    rethrow(ME);
end
