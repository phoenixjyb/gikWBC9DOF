function absPath = resolvePath(target, expectDir)
%RESOLVEPATH Resolve a relative asset location against the project root.
%   absPath = RESOLVEPATH(target) returns an absolute path. When target is
%   already absolute, it is returned unchanged. Relative paths are resolved
%   against the repository root so callers can reference assets with
%   project-scoped paths.
%
%   absPath = RESOLVEPATH(target, expectDir) lets the caller state whether a
%   directory is expected (true) or a file (false, default). The helper does
%   not create folders; it only converts the path into an absolute form.

arguments
    target (1,1) string
    expectDir (1,1) logical = false
end

if isAbsolutePath(target)
    absPath = target;
    return
end

root = gik9dof.internal.projectRoot();
absPath = fullfile(root, target);

if expectDir
    absPath = string(absPath);
else
    absPath = string(absPath);
end
end

function tf = isAbsolutePath(pathStr)
%ISABSOLUTEPATH Detect platform specific absolute paths.
if startsWith(pathStr, filesep)
    tf = true;
    return
end
if ispc && contains(pathStr, ":")
    tf = true;
    return
end
if startsWith(pathStr, "\\")
    tf = true;
    return
end
if contains(pathStr, "://") || startsWith(pathStr, "classpath:")
    tf = true;
    return
end

tf = false;
end
