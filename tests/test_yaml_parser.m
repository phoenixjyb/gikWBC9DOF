% Test the custom YAML parser with debugging
addpath('matlab');

yamlPath = 'config/pipeline_profiles.yaml';
rawLines = string(splitlines(fileread(yamlPath)));

data = struct('profiles', struct());
stack = {};
currentStruct = data;

fprintf('Parsing YAML file...\n\n');

for i = 1:min(50, numel(rawLines))  % Just parse first 50 lines for debugging
    rawLine = rawLines(i);
    line = strtrim(rawLine);
    
    % Skip empty lines and comments
    if line == "" || startsWith(line, "#")
        continue
    end
    
    % Calculate indentation
    if strlength(rawLine) == 0
        indent = 0;
    else
        charLine = char(rawLine);
        indent = 0;
        for j = 1:length(charLine)
            if charLine(j) == ' ' || charLine(j) == sprintf('\t')
                indent = indent + 1;
            else
                break;
            end
        end
    end
    
    % Check if this is a key-value pair or just a key
    if contains(line, ":")
        parts = split(line, ':', 2);
        key = char(strtrim(parts(1)));
        
        if numel(parts) == 2 && strtrim(parts(2)) ~= ""
            % Key-value pair
            valueStr = strtrim(parts(2));
            fprintf('[%3d] indent=%2d KEY-VALUE: "%s" = "%s"\n', i, indent, key, valueStr);
        else
            % Just a key (nested struct marker)
            fprintf('[%3d] indent=%2d KEY ONLY:   "%s"\n', i, indent, key);
        end
    end
end

fprintf('\n\nNow let us see what loadPipelineProfile returns:\n');
try
    cfg = gik9dof.loadPipelineProfile('default');
    disp('SUCCESS! Loaded profile');
    disp(cfg);
catch ME
    disp(['ERROR: ' ME.message]);
    if ~isempty(ME.stack)
        disp(['  at line ' num2str(ME.stack(1).line)]);
    end
end
