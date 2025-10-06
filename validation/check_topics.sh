#!/bin/bash
# Test if state messages are actually being published

cd ~/gikWBC9DOF/ros2
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "Testing state message publishing..."
echo ""

# Start test publisher
cd ~/gikWBC9DOF/validation
python3 simple_test.py &
TEST_PID=$!

# Wait a moment for publishers to start
sleep 4

# Echo topics
echo "=== Checking /odom_wheel ==="
timeout 2 ros2 topic echo /odom_wheel --once

echo ""
echo "=== Checking /hdas/feedback_arm_left ==="
timeout 2 ros2 topic echo /hdas/feedback_arm_left --once

echo ""
echo "=== Checking /gik9dof/target_trajectory ==="
timeout 2 ros2 topic echo /gik9dof/target_trajectory --once

# Kill test
kill $TEST_PID 2>/dev/null
wait $TEST_PID 2>/dev/null

echo ""
echo "Done."
