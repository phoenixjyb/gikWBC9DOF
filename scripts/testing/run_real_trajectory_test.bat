@echo off
REM Test trajectory smoothing with real waypoint data
REM This script launches MATLAB and runs the test

echo ========================================
echo Real Trajectory Smoothing Test
echo ========================================
echo.
echo Loading 300 waypoints from 1_pull_world_scaled.json
echo Testing S-curve smoothing at 50Hz
echo.

cd /d "%~dp0"

REM Run MATLAB test
matlab -batch "cd matlab; test_smoothing_real_data"

echo.
echo ========================================
echo Test complete!
echo Check the MATLAB figure for results
echo ========================================
pause
