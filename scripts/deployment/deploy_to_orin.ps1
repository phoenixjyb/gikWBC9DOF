# Deploy to AGX Orin - Unified Deployment Script
#
# Usage: .\deploy_to_orin.ps1 -OrinIP <IP> -Mode <Complete|ChassisOnly>

param(
    [Parameter(Mandatory=$true)]
    [string]$OrinIP,
    
    [Parameter(Mandatory=$false)]
    [string]$Username = "cr",
    
    [Parameter(Mandatory=$false)]
    [string]$RemotePath = "/home/nvidia/temp_gikrepo",
    
    [Parameter(Mandatory=$false)]
    [ValidateSet("Complete", "ChassisOnly")]
    [string]$Mode = "Complete"
)

Write-Host "Deployment script started" -ForegroundColor Green
Write-Host "Target: $Username@$OrinIP" -ForegroundColor Cyan
Write-Host "Mode: $Mode" -ForegroundColor Cyan
