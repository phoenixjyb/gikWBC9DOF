#!/bin/bash
# Simple manual test - publish using ros2 topic pub

cd ~/gikWBC9DOF/ros2
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "Starting solver..."
ros2 run gik9dof_solver gik9dof_solver_node &
SOLVER_PID=$!

sleep 3

echo ""
echo "Publishing odom using ros2 tool..."
ros2 topic pub --once /odom_wheel nav_msgs/msg/Odometry "{header: {frame_id: 'odom'}, pose: {pose: {position: {x: 1.65, y: 0.08, z: 0.0}, orientation: {w: 1.0}}}}" &

sleep 1

echo ""
echo "Publishing arm state using ros2 tool..."
ros2 topic pub --once /hdas/feedback_arm_left sensor_msgs/msg/JointState "{name: ['j1','j2','j3','j4','j5','j6'], position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}" &

sleep 5

echo ""
echo "Killing solver..."
kill $SOLVER_PID 2>/dev/null
wait 2>/dev/null

echo "Done."
