function params = loadChassisProfile(profileName, options)
%LOADCHASSISPROFILE Load chassis controller parameters from YAML presets.
%   params = gik9dof.control.loadChassisProfile(name) loads the profile named
%   NAME from config/chassis_profiles.yaml and returns a struct with fields
%   matching the YAML entries. Optional overrides can be supplied via
%   name-value pairs:
%       FilePath  - alternate YAML file (default config/chassis_profiles.yaml)
%       Overrides - struct of fields to merge onto the selected profile
%
%   Example:
%       params = gik9dof.control.loadChassisProfile("wide_track");
%       params.vx_max = 1.2;  % manual tweak if needed
%
%   The function raises an error when the profile is missing or YAML support
%   is unavailable.

arguments
    profileName (1,1) string
    options.FilePath (1,1) string = ""
    options.Overrides (1,1) struct = struct()
end

if strlength(options.FilePath) > 0
    yamlPath = options.FilePath;
else
    yamlPath = fullfile(gik9dof.internal.projectRoot(), "config", "chassis_profiles.yaml");
end

if ~isfile(yamlPath)
    error("gik9dof:loadChassisProfile:MissingFile", ...
        "Chassis profile YAML not found: %s", yamlPath);
end

profileData = readYamlFile(yamlPath);

if ~isfield(profileData, "profiles")
    error("gik9dof:loadChassisProfile:MissingProfiles", ...
        "YAML file %s does not contain a 'profiles' section.", yamlPath);
end

profiles = profileData.profiles;
if ~isfield(profiles, profileName)
    available = string(fieldnames(profiles));
    error("gik9dof:loadChassisProfile:UnknownProfile", ...
        "Profile '%s' not found in %s. Available profiles: %s", ...
        profileName, yamlPath, strjoin(available, ", "));
end

params = profiles.(profileName);

% Merge overrides (shallow).
overrideFields = fieldnames(options.Overrides);
for k = 1:numel(overrideFields)
    fieldName = overrideFields{k};
    params.(fieldName) = options.Overrides.(fieldName);
end

params.profile = profileName;
params.sourceFile = yamlPath;
end

function data = readYamlFile(yamlPath)
%READYAMLFILE Load YAML file using built-in or fallback mechanisms.
if exist("yamlread", "file") == 2
    data = yamlread(yamlPath);
elseif exist("matlab.internal.yaml.loadFile", "file") == 2
    data = matlab.internal.yaml.loadFile(yamlPath);
else
    data = readYamlSimple(yamlPath);
end

% Convert string scalars to double/struct as needed (yamlread already does).
if ~isstruct(data)
    error("gik9dof:loadChassisProfile:InvalidYaml", ...
        "Unexpected data type returned when loading %s.", yamlPath);
end
end

function data = readYamlSimple(yamlPath)
%READYAMLSIMPLE Minimal parser for simple key/value YAML files.
rawLines = string(splitlines(fileread(yamlPath)));

profilesStruct = struct();
currentProfile = "";
currentSection = "";

for i = 1:numel(rawLines)
    rawLine = rawLines(i);
    line = strtrim(rawLine);
    if line == "" || startsWith(line, "#")
        continue
    end

    indent = strlength(rawLine) - strlength(strip(rawLine, 'left'));

    if endsWith(line, ":")
        key = extractBefore(line, strlength(line));
        if indent == 0
            if key ~= "profiles"
                error("gik9dof:loadChassisProfile:UnsupportedYaml", ...
                    "Unexpected top-level key '%s' in %s.", key, yamlPath);
            end
            continue
        elseif indent == 2
            currentProfile = string(key);
            profilesStruct.(currentProfile) = struct();
            currentSection = "";
        elseif indent == 4
            currentSection = string(key);
            profilesStruct.(currentProfile).(currentSection) = struct();
        else
            error("gik9dof:loadChassisProfile:UnsupportedYaml", ...
                "Unsupported indentation at line %d in %s.", i, yamlPath);
        end
        continue
    end

    parts = split(line, ':');
    if numel(parts) < 2
        continue
    end
key = strtrim(parts(1));
valueStr = strtrim(strjoin(parts(2:end), ':'));
hashPos = strfind(valueStr, '#');
if ~isempty(hashPos)
    valueStr = strtrim(extractBefore(valueStr, hashPos(1)));
end
value = convertScalar(valueStr);

    if indent == 4 && currentSection == ""
        profilesStruct.(currentProfile).(key) = value;
    elseif indent >= 6 && currentSection ~= ""
        profilesStruct.(currentProfile).(currentSection).(key) = value;
    else
        error("gik9dof:loadChassisProfile:UnsupportedYaml", ...
            "Unexpected structure around line %d in %s.", i, yamlPath);
    end
end

data = struct('profiles', profilesStruct);
end

function value = convertScalar(valueStr)
if valueStr == "true"
    value = true;
elseif valueStr == "false"
    value = false;
else
    numVal = str2double(valueStr);
    if ~isnan(numVal)
        value = numVal;
    else
        value = string(valueStr);
    end
end
end
