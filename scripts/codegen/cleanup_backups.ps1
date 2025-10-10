# Clean Codegen Backup Folders
# Safely removes dated backup folders that can be regenerated

Write-Host "========================================"  -ForegroundColor Cyan
Write-Host "Codegen Backup Cleanup" -ForegroundColor Cyan
Write-Host "========================================" -ForegroundColor Cyan
Write-Host ""

$backupFolders = @(
    "codegen\planner_arm64_backup_20251009_132832",
    "codegen\planner_arm64_backup_20251009_132850"
)

Write-Host "Folders to be removed:" -ForegroundColor Yellow
foreach ($folder in $backupFolders) {
    if (Test-Path $folder) {
        $size = (Get-ChildItem $folder -Recurse | Measure-Object -Property Length -Sum).Sum / 1MB
        Write-Host "  - $folder (${size:N2} MB)" -ForegroundColor White
    } else {
        Write-Host "  - $folder (not found)" -ForegroundColor Gray
    }
}

Write-Host ""
Write-Host "Folders that will be KEPT:" -ForegroundColor Green
Write-Host "  - codegen\archive\ (historical versions)" -ForegroundColor White
Write-Host "  - codegen\arm64_realtime\ (active)" -ForegroundColor White
Write-Host "  - codegen\planner_arm64\ (active)" -ForegroundColor White
Write-Host "  - codegen\trajectory_smoothing\ (active)" -ForegroundColor White
Write-Host "  - codegen\velocity_smoothing\ (active)" -ForegroundColor White

Write-Host ""
$confirmation = Read-Host "Proceed with cleanup? (yes/no)"

if ($confirmation -eq "yes") {
    Write-Host ""
    Write-Host "Removing backup folders..." -ForegroundColor Yellow
    
    foreach ($folder in $backupFolders) {
        if (Test-Path $folder) {
            try {
                Remove-Item $folder -Recurse -Force
                Write-Host "  ✓ Removed: $folder" -ForegroundColor Green
            } catch {
                Write-Host "  ✗ Failed to remove: $folder" -ForegroundColor Red
                Write-Host "    Error: $_" -ForegroundColor Red
            }
        } else {
            Write-Host "  ⊘ Already removed: $folder" -ForegroundColor Gray
        }
    }
    
    Write-Host ""
    Write-Host "✅ Cleanup complete!" -ForegroundColor Green
    
    # Show remaining structure
    Write-Host ""
    Write-Host "Remaining codegen structure:" -ForegroundColor Cyan
    Get-ChildItem codegen -Directory | ForEach-Object {
        $size = (Get-ChildItem $_.FullName -Recurse -File | Measure-Object -Property Length -Sum).Sum / 1MB
        $fileCount = (Get-ChildItem $_.FullName -Recurse -File).Count
        Write-Host "  $($_.Name)" -ForegroundColor White -NoNewline
        Write-Host " - $fileCount files, ${size:N2} MB" -ForegroundColor Gray
    }
    
} else {
    Write-Host ""
    Write-Host "Cleanup cancelled." -ForegroundColor Yellow
    Write-Host "No files were removed." -ForegroundColor Gray
}

Write-Host ""
Write-Host "Note: These backups can be safely deleted because:" -ForegroundColor Gray
Write-Host "  1. They are dated snapshots that can be regenerated" -ForegroundColor Gray
Write-Host "  2. Current active code is in non-backup folders" -ForegroundColor Gray
Write-Host "  3. Git history preserves all changes" -ForegroundColor Gray
Write-Host "  4. archive/ folder contains historical versions if needed" -ForegroundColor Gray
