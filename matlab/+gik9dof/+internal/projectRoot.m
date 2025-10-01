function root = projectRoot()
%PROJECTROOT Return the repository root directory.
%   The helper caches the location to avoid repeated filesystem queries and
%   is used to resolve default asset paths.

persistent cachedRoot
if ~isempty(cachedRoot) && isfolder(cachedRoot)
    root = cachedRoot;
    return
end

thisFile = mfilename("fullpath");
% projectRoot.m -> +internal -> +gik9dof -> matlab -> <repo>
root = fileparts(fileparts(fileparts(fileparts(thisFile))));

if ~isfolder(root)
    error("gik9dof:internal:projectRoot:ResolutionFailed", ...
        "Unable to determine repository root starting from %s", thisFile);
end

cachedRoot = root;
end
