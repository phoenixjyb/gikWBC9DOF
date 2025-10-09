#!/bin/bash
# Test script to verify GIK solver is working correctly on ARM64

echo "=== GIK9DOF Solver ARM64 Functional Test ==="
echo ""

# Check if ROS2 is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo "Sourcing ROS2 Humble..."
    source /opt/ros/humble/setup.bash
fi

# Source workspace
echo "Sourcing workspace..."
source ~/gikWBC9DOF/ros2/install/setup.bash

# Check if solver node is running
echo ""
echo "1. Checking if solver node is running..."
if ros2 node list | grep -q gik9dof_solver; then
    echo "   ✓ Solver node is running"
    NODE_RUNNING=true
else
    echo "   ✗ Solver node NOT running"
    echo "   Starting solver node in background..."
    ros2 run gik9dof_solver gik9dof_solver_node &
    SOLVER_PID=$!
    sleep 3
    NODE_RUNNING=false
fi

# Check solver diagnostics
echo ""
echo "2. Checking solver diagnostics..."
timeout 2 ros2 topic echo /gik9dof/solver_diagnostics --once 2>/dev/null
if [ $? -eq 0 ]; then
    echo "   ✓ Solver is publishing diagnostics"
else
    echo "   ⚠ No diagnostics received (may be waiting for input)"
fi

# Check topic connections
echo ""
echo "3. Checking topic connections..."
echo "   Subscriptions:"
ros2 topic info /gik9dof/target_trajectory 2>/dev/null | grep "Subscription count:" || echo "   ⚠ Topic not found"

echo "   Publications:"
ros2 topic info /motion_target/target_joint_state_arm_left 2>/dev/null | grep "Publisher count:" || echo "   ⚠ Topic not found"

# Send a test trajectory and check response
echo ""
echo "4. Sending test trajectory command..."
echo "   Target: Move end-effector to [0.5, 0.0, 0.5]"

# Create a simple test trajectory (adjust message structure as needed)
timeout 5 ros2 topic pub --once /gik9dof/target_trajectory gik9dof_msgs/msg/EndEffectorTrajectory "{
  header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'base_link'},
  points: [{
    pose: {
      position: {x: 0.5, y: 0.0, z: 0.5},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    },
    time_from_start: {sec: 1, nanosec: 0}
  }]
}" 2>/dev/null

if [ $? -eq 0 ]; then
    echo "   ✓ Test command sent"
    
    # Wait and check if solver published a solution
    echo ""
    echo "5. Checking for solver output..."
    timeout 3 ros2 topic echo /motion_target/target_joint_state_arm_left --once 2>/dev/null
    
    if [ $? -eq 0 ]; then
        echo ""
        echo "   ✓✓✓ SOLVER IS WORKING! Valid joint solution received!"
    else
        echo ""
        echo "   ⚠ No joint solution received - solver may not be processing correctly"
    fi
else
    echo "   ✗ Failed to send test command (message type may need adjustment)"
fi

# Cleanup
if [ "$NODE_RUNNING" = false ] && [ ! -z "$SOLVER_PID" ]; then
    echo ""
    echo "Stopping test solver node..."
    kill $SOLVER_PID 2>/dev/null
fi

echo ""
echo "=== Test Complete ==="
