#!/bin/bash
# Full integration test - runs solver and test in same ROS environment

cd ~/gikWBC9DOF/ros2
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "========================================"
echo "Starting Full Integration Test"
echo "========================================"

# Start solver node in background
echo "[1/3] Starting solver node..."
ros2 run gik9dof_solver gik9dof_solver_node &
SOLVER_PID=$!
echo "Solver PID: $SOLVER_PID"

# Wait for solver to initialize
sleep 3

# Run test
echo "[2/3] Running test script..."
cd ~/gikWBC9DOF/validation
python3 simple_test.py
TEST_RESULT=$?

# Kill solver
echo "[3/3] Stopping solver node..."
kill $SOLVER_PID
wait $SOLVER_PID 2>/dev/null

echo "========================================"
if [ $TEST_RESULT -eq 0 ]; then
    echo "✅ TEST PASSED"
else
    echo "❌ TEST FAILED"
fi
echo "========================================"

exit $TEST_RESULT
