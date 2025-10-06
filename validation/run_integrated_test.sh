#!/bin/bash
# Run solver and test with both outputs visible

echo "========================================"
echo "GIK9DOF Solver Integration Test"
echo "========================================"

# Setup environment
cd ~/gikWBC9DOF/ros2
source /opt/ros/humble/setup.bash
source install/setup.bash

# Ensure ROS_DOMAIN_ID is set (default to 0)
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
echo "Using ROS_DOMAIN_ID: $ROS_DOMAIN_ID"

# Start solver node
echo ""
echo "[1/4] Starting solver node..."
ros2 run gik9dof_solver gik9dof_solver_node > /tmp/solver_output.log 2>&1 &
SOLVER_PID=$!
echo "      Solver PID: $SOLVER_PID"
echo "      Logging to: /tmp/solver_output.log"

# Wait for solver to initialize and start advertising topics
echo ""
echo "[2/4] Waiting for solver to initialize (5 seconds for DDS discovery)..."
sleep 5

# Check if solver is still running
if ! ps -p $SOLVER_PID > /dev/null; then
    echo "ERROR: Solver crashed during startup!"
    cat /tmp/solver_output.log
    exit 1
fi

# Show solver status
echo ""
echo "[3/4] Solver startup log:"
echo "----------------------------------------"
head -20 /tmp/solver_output.log
echo "----------------------------------------"

# Check topics
echo ""
echo "Active ROS2 topics:"
ros2 topic list
echo ""

# Run test in SAME shell (same ROS environment)
echo "[4/4] Running test script..."
cd ~/gikWBC9DOF/validation
python3 simple_test.py &
TEST_PID=$!

# Wait for test to complete
wait $TEST_PID
TEST_RESULT=$?

# Show recent solver output
echo ""
echo "========================================"
echo "Solver output during test:"
echo "========================================"
tail -40 /tmp/solver_output.log

# Cleanup
echo ""
echo "Stopping solver node (PID: $SOLVER_PID)..."
kill $SOLVER_PID 2>/dev/null
wait $SOLVER_PID 2>/dev/null

echo ""
echo "========================================"
if [ $TEST_RESULT -eq 0 ]; then
    echo "✅ TEST PASSED"
else
    echo "❌ TEST FAILED"
fi
echo "========================================"

exit $TEST_RESULT
