#!/bin/bash
# Check QoS profiles of topics

cd ~/gikWBC9DOF/ros2
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "Starting solver..."
ros2 run gik9dof_solver gik9dof_solver_node &
SOLVER_PID=$!
sleep 2

echo ""
echo "Starting test publisher..."
cd ~/gikWBC9DOF/validation
python3 simple_test.py &
TEST_PID=$!
sleep 3

echo ""
echo "=== Topic: /odom_wheel ==="
ros2 topic info /odom_wheel -v

echo ""
echo "=== Topic: /hdas/feedback_arm_left ==="
ros2 topic info /hdas/feedback_arm_left -v

echo ""
echo "Cleaning up..."
kill $TEST_PID $SOLVER_PID 2>/dev/null
wait 2>/dev/null

