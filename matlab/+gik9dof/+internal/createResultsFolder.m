function runDir = createResultsFolder(label)
%CREATERESULTSFOLDER Create timestamped directory under results/.
%   runDir = gik9dof.internal.createResultsFolder(label) creates a
%   subdirectory inside the repository-level results folder named using the
%   current timestamp and optional label. The function ensures both the root
%   results directory and the timestamped subfolder exist and returns the
%   absolute path.
%
%   label may be empty. Invalid filesystem characters are replaced with
%   underscores.

if nargin < 1 || strlength(label) == 0
    label = "";
else
    label = sanitizeLabel(label);
end

timestamp = string(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));

if strlength(label) > 0
    folderName = timestamp + "_" + label;
else
    folderName = timestamp;
end

root = gik9dof.internal.projectRoot();
resultsRoot = fullfile(root, "results");
if ~isfolder(resultsRoot)
    mkdir(resultsRoot);
end

runDir = fullfile(resultsRoot, folderName);
if ~isfolder(runDir)
    mkdir(runDir);
end
runDir = string(runDir);
end

function labelOut = sanitizeLabel(labelIn)
labelChars = char(labelIn);
labelChars(~isstrprop(labelChars, 'alphanum') & labelChars ~= '-' & labelChars ~= '_') = '_';
labelOut = string(labelChars);
end
