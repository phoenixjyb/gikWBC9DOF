# Get Build Information Script
# Generates a build info file with git commit, timestamp, and other metadata

param(
    [string]$OutputFile = "BUILD_INFO.txt"
)

Write-Host "Generating build information..." -ForegroundColor Cyan

# Get git information
$gitCommit = git rev-parse HEAD
$gitCommitShort = git rev-parse --short HEAD
$gitBranch = git rev-parse --abbrev-ref HEAD
$gitIsDirty = (git status --porcelain).Length -gt 0

# Get timestamp
$buildTime = Get-Date -Format "yyyy-MM-dd HH:mm:ss"
$buildTimestamp = Get-Date -Format "yyyyMMdd_HHmmss"

# Get user/machine info
$buildUser = $env:USERNAME
$buildMachine = $env:COMPUTERNAME

# Create build info content
$buildInfo = @"
===========================================
BUILD INFORMATION
===========================================

Build Time:     $buildTime
Build ID:       $buildTimestamp
Git Commit:     $gitCommit
Git Short:      $gitCommitShort
Git Branch:     $gitBranch
Git Status:     $(if ($gitIsDirty) { "DIRTY (uncommitted changes)" } else { "CLEAN" })
Built By:       $buildUser@$buildMachine

===========================================
VERIFICATION
===========================================

To verify this build matches your source:
  git checkout $gitCommitShort

To check if source has changed since build:
  git diff $gitCommitShort

===========================================
"@

# Write to file
$buildInfo | Out-File -FilePath $OutputFile -Encoding UTF8

Write-Host "✓ Build info written to: $OutputFile" -ForegroundColor Green
Write-Host ""
Write-Host "Build ID: $buildTimestamp" -ForegroundColor Yellow
Write-Host "Git Commit: $gitCommitShort" -ForegroundColor Yellow

if ($gitIsDirty) {
    Write-Host ""
    Write-Host "⚠️ WARNING: Uncommitted changes in workspace!" -ForegroundColor Red
    Write-Host "Consider committing before building for deployment" -ForegroundColor Yellow
}
