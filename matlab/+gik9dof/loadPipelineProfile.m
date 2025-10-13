function config = loadPipelineProfile(profileName, options)
%LOADPIPELINEPROFILE Load unified pipeline configuration from YAML profiles.
%   config = gik9dof.loadPipelineProfile(name) loads the profile named NAME
%   from config/pipeline_profiles.yaml and returns a struct with all pipeline
%   parameters organized hierarchically. Supports profile inheritance and
%   field-level overrides.
%
%   config = gik9dof.loadPipelineProfile(name, 'Overrides', overrideStruct)
%   merges OVERRIDESTRUCT onto the loaded profile (deep merge).
%
%   config = gik9dof.loadPipelineProfile(name, 'FilePath', yamlPath)
%   loads from an alternate YAML file instead of the default.
%
%   config = gik9dof.loadPipelineProfile(name, 'Validate', true)
%   performs consistency checks on the loaded configuration (default: true).
%
%   EXAMPLES:
%       % Load default profile
%       cfg = gik9dof.loadPipelineProfile('default');
%
%       % Load aggressive profile with custom override
%       cfg = gik9dof.loadPipelineProfile('aggressive', ...
%           'Overrides', struct('stage_b', struct('desired_linear_velocity', 1.0)));
%
%       % Access parameters
%       trackWidth = cfg.chassis.track;
%       stageBVel = cfg.stage_b.desired_linear_velocity;
%
%   The function resolves profile inheritance ('inherits' field) and applies
%   overrides hierarchically. It validates parameter consistency (e.g., ensures
%   stage_c.track_width matches chassis.track) and issues warnings for
%   inconsistencies.
%
%   See also gik9dof.control.loadChassisProfile, gik9dof.control.defaultUnifiedParams

arguments
    profileName (1,1) string
    options.FilePath (1,1) string = ""
    options.Overrides (1,1) struct = struct()
    options.Validate (1,1) logical = true
end

% Resolve YAML file path
if strlength(options.FilePath) > 0
    yamlPath = options.FilePath;
else
    yamlPath = fullfile(gik9dof.internal.projectRoot(), "config", "pipeline_profiles.yaml");
end

% Convert YAML path to JSON path
[folder, name, ~] = fileparts(yamlPath);
jsonPath = fullfile(folder, name + ".json");

% Check if JSON exists, if not provide helpful error
if ~isfile(jsonPath)
    error("gik9dof:loadPipelineProfile:MissingJSON", ...
        ["JSON configuration file not found: %s\n\n" ...
         "Please run the YAML-to-JSON converter:\n" ...
         "  python3 utils/yaml_to_json.py\n\n" ...
         "This converts config/pipeline_profiles.yaml to pipeline_profiles.json"], ...
        jsonPath);
end

% Load JSON file
profileData = readJsonFile(jsonPath);

if ~isfield(profileData, "profiles")
    error("gik9dof:loadPipelineProfile:MissingProfiles", ...
        "YAML file %s does not contain a 'profiles' section.", yamlPath);
end

profiles = profileData.profiles;

% Resolve profile with inheritance
config = resolveProfile(profiles, profileName, yamlPath);

% Apply user overrides (deep merge)
config = deepMergeStructs(config, options.Overrides);

% Add metadata
config.meta = struct();
config.meta.profile = profileName;
config.meta.sourceFile = yamlPath;
config.meta.loadedAt = datetime('now');

% Validate configuration
if options.Validate
    config = validateConfig(config);
end

end

%% Helper Functions

function config = resolveProfile(profiles, profileName, yamlPath)
%RESOLVEPROFILE Resolve profile with inheritance chain.

if ~isfield(profiles, profileName)
    available = string(fieldnames(profiles));
    error("gik9dof:loadPipelineProfile:UnknownProfile", ...
        "Profile '%s' not found in %s.\nAvailable profiles: %s", ...
        profileName, yamlPath, strjoin(available, ", "));
end

profile = profiles.(profileName);

% Check for circular inheritance
visited = string([]);
config = resolveProfileRecursive(profiles, profileName, visited);

% Remove inheritance metadata from final config
if isfield(config, 'inherits')
    config = rmfield(config, 'inherits');
end
if isfield(config, 'overrides')
    config = rmfield(config, 'overrides');
end

end

function config = resolveProfileRecursive(profiles, profileName, visited)
%RESOLVEPROFILERECURSIVE Recursively resolve profile inheritance.

% Check for circular dependency
if ismember(profileName, visited)
    error("gik9dof:loadPipelineProfile:CircularInheritance", ...
        "Circular inheritance detected: %s", strjoin([visited, profileName], " -> "));
end

visited = [visited, profileName];
profile = profiles.(profileName);

% Base case: no inheritance
if ~isfield(profile, 'inherits')
    config = profile;
    return
end

% Recursive case: inherit from parent
parentName = string(profile.inherits);
parentConfig = resolveProfileRecursive(profiles, parentName, visited);

% Merge parent with current profile
if isfield(profile, 'overrides')
    config = deepMergeStructs(parentConfig, profile.overrides);
else
    config = parentConfig;
end

% Copy over description if present
if isfield(profile, 'description')
    config.description = profile.description;
end

end

function merged = deepMergeStructs(base, overlay)
%DEEPMERGESTRUCTS Deep merge two structs (overlay takes precedence).
%   For each field in overlay:
%     - If field is struct in both, recursively merge
%     - Otherwise, overlay value replaces base value

merged = base;
overlayFields = fieldnames(overlay);

for i = 1:numel(overlayFields)
    fieldName = overlayFields{i};
    overlayValue = overlay.(fieldName);
    
    if isfield(merged, fieldName) && isstruct(merged.(fieldName)) && isstruct(overlayValue)
        % Both are structs -> recursive merge
        merged.(fieldName) = deepMergeStructs(merged.(fieldName), overlayValue);
    else
        % Overlay replaces base
        merged.(fieldName) = overlayValue;
    end
end

end

function config = validateConfig(config)
%VALIDATECONFIG Check for parameter inconsistencies and issue warnings.

warnings = string([]);

% Check chassis vs stage_c consistency
if isfield(config, 'chassis') && isfield(config, 'stage_c')
    if isfield(config.chassis, 'track') && isfield(config.stage_c, 'track_width')
        if abs(config.chassis.track - config.stage_c.track_width) > 1e-6
            warnings = [warnings, sprintf( ...
                "stage_c.track_width (%.4f) differs from chassis.track (%.4f). Using chassis.track.", ...
                config.stage_c.track_width, config.chassis.track)];
            config.stage_c.track_width = config.chassis.track;
        end
    end
    
    if isfield(config.chassis, 'wheel_base') && isfield(config.stage_c, 'wheel_base')
        if abs(config.chassis.wheel_base - config.stage_c.wheel_base) > 1e-6
            warnings = [warnings, sprintf( ...
                "stage_c.wheel_base (%.4f) differs from chassis.wheel_base (%.4f). Using chassis.wheel_base.", ...
                config.stage_c.wheel_base, config.chassis.wheel_base)];
            config.stage_c.wheel_base = config.chassis.wheel_base;
        end
    end
    
    if isfield(config.chassis, 'wheel_speed_max') && isfield(config.stage_c, 'max_wheel_speed')
        if abs(config.chassis.wheel_speed_max - config.stage_c.max_wheel_speed) > 1e-6
            warnings = [warnings, sprintf( ...
                "stage_c.max_wheel_speed (%.4f) differs from chassis.wheel_speed_max (%.4f). Using chassis.wheel_speed_max.", ...
                config.stage_c.max_wheel_speed, config.chassis.wheel_speed_max)];
            config.stage_c.max_wheel_speed = config.chassis.wheel_speed_max;
        end
    end
end

% Check velocity limits consistency
if isfield(config, 'chassis') && isfield(config, 'stage_b')
    if isfield(config.chassis, 'vx_max') && isfield(config.stage_b, 'max_linear_speed')
        if config.stage_b.max_linear_speed > config.chassis.vx_max
            warnings = [warnings, sprintf( ...
                "stage_b.max_linear_speed (%.2f) exceeds chassis.vx_max (%.2f). Clamping to chassis limit.", ...
                config.stage_b.max_linear_speed, config.chassis.vx_max)];
            config.stage_b.max_linear_speed = config.chassis.vx_max;
        end
    end
end

if isfield(config, 'chassis') && isfield(config, 'stage_c')
    if isfield(config.chassis, 'vx_max') && isfield(config.stage_c, 'max_linear_speed')
        if config.stage_c.max_linear_speed > config.chassis.vx_max
            warnings = [warnings, sprintf( ...
                "stage_c.max_linear_speed (%.2f) exceeds chassis.vx_max (%.2f). Clamping to chassis limit.", ...
                config.stage_c.max_linear_speed, config.chassis.vx_max)];
            config.stage_c.max_linear_speed = config.chassis.vx_max;
        end
    end
end

% Issue warnings if any
if ~isempty(warnings)
    warning("gik9dof:loadPipelineProfile:InconsistentParameters", ...
        "Configuration inconsistencies detected:\n  %s", strjoin(warnings, "\n  "));
end

% Store validation results
config.meta.validationWarnings = warnings;

end

function data = readJsonFile(jsonPath)
%READJSONFILE Load JSON configuration file.
%   Uses MATLAB's built-in jsondecode function to read JSON configuration.
%   The JSON file should be generated from YAML using utils/yaml_to_json.py

try
    jsonText = fileread(jsonPath);
    data = jsondecode(jsonText);
catch ME
    error("gik9dof:loadPipelineProfile:InvalidJson", ...
        "Failed to parse JSON file %s: %s", jsonPath, ME.message);
end

if ~isstruct(data)
    error("gik9dof:loadPipelineProfile:InvalidJson", ...
        "JSON file %s did not produce a struct.", jsonPath);
end

end
