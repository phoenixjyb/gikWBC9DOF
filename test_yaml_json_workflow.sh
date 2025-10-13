#!/bin/bash
# Complete workflow test: Edit YAML → Convert → Test in MATLAB

set -e  # Exit on error

echo "=== YAML → JSON → MATLAB Workflow Test ==="
echo

# Step 1: Verify YAML exists
echo "Step 1: Checking YAML source..."
if [ -f "config/pipeline_profiles.yaml" ]; then
    echo "  ✓ config/pipeline_profiles.yaml found"
else
    echo "  ✗ YAML file missing!"
    exit 1
fi

# Step 2: Convert YAML to JSON
echo
echo "Step 2: Converting YAML to JSON..."

# Use venv if available, otherwise system python
if [ -f ".venv/bin/python" ]; then
    PYTHON=".venv/bin/python"
else
    PYTHON="python3"
fi

$PYTHON utils/yaml_to_json.py
if [ $? -eq 0 ]; then
    echo "  ✓ Conversion successful"
else
    echo "  ✗ Conversion failed!"
    exit 1
fi

# Step 3: Verify JSON was created
echo
echo "Step 3: Verifying JSON output..."
if [ -f "config/pipeline_profiles.json" ]; then
    SIZE=$(wc -c < config/pipeline_profiles.json)
    echo "  ✓ config/pipeline_profiles.json exists ($SIZE bytes)"
else
    echo "  ✗ JSON file not created!"
    exit 1
fi

# Step 4: Test loading in MATLAB
echo
echo "Step 4: Testing MATLAB profile loading..."
matlab -batch "addpath('matlab'); \
    try \
        cfg = gik9dof.loadPipelineProfile('aggressive'); \
        disp('  ✓ Aggressive profile loaded'); \
        fprintf('    - yaw_corridor: %d°\n', cfg.stage_c.ppfirst.yaw_corridor_deg); \
        fprintf('    - position_tol: %.3fm\n', cfg.stage_c.ppfirst.position_tolerance); \
        fprintf('    - ee_threshold: %.4fm\n', cfg.stage_c.ppfirst.ee_error_threshold); \
        exit(0); \
    catch ME; \
        fprintf('  ✗ FAILED: %s\n', ME.message); \
        exit(1); \
    end" 2>&1 | grep -E "(✓|✗|-)"

if [ ${PIPESTATUS[0]} -eq 0 ]; then
    echo
    echo "=== All Tests Passed ==="
    echo
    echo "Workflow is operational:"
    echo "  1. Edit: config/pipeline_profiles.yaml"
    echo "  2. Convert: python3 utils/yaml_to_json.py"
    echo "  3. Use: gik9dof.loadPipelineProfile('profile_name')"
else
    echo
    echo "=== Test Failed ==="
    exit 1
fi
