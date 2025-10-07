#!/bin/bash
# Test script for trajectory completion feature

echo "==================================="
echo "Trajectory Completion Test"
echo "==================================="

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}Step 1: Source ROS2${NC}"
source /opt/ros/humble/setup.bash
cd /mnt/c/Users/yanbo/wSpace/codegenGIKsample/Trial/gikWBC9DOF/ros2
source install/setup.bash

echo -e "${BLUE}Step 2: Publish joint states in background${NC}"
ros2 topic pub /joint_states sensor_msgs/msg/JointState \
"{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'base_link'}, 
  name: ['base_x', 'base_y', 'base_theta', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'], 
  position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}" \
--rate 10 &

JOINT_PUB_PID=$!
echo "Joint states publisher started (PID: $JOINT_PUB_PID)"

echo -e "${BLUE}Step 3: Wait for node to be ready (5 seconds)${NC}"
sleep 5

echo -e "${GREEN}Step 4: Publishing test trajectory with 3 waypoints${NC}"
ros2 topic pub --once /gik9dof/desired_ee_trajectory gik9dof_msgs/msg/EndEffectorTrajectory \
"{
  header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'map'},
  waypoints: [
    {header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'map'}, 
     pose: {position: {x: 1.0, y: 0.0, z: 0.5}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}},
    {header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'map'}, 
     pose: {position: {x: 1.5, y: 0.0, z: 0.5}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}},
    {header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'map'}, 
     pose: {position: {x: 2.0, y: 0.0, z: 0.5}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}
  ],
  timestamps: [0.0, 1.0, 2.0],
  sequence_id: 1,
  is_final_segment: true,
  lookahead_time: 2.0,
  max_velocity: 0.5,
  max_acceleration: 0.2
}"

echo ""
echo -e "${GREEN}✓ Test trajectory sent!${NC}"
echo ""
echo "Expected behavior:"
echo "  1. Node tracks waypoint 0 (x=1.0)"
echo "  2. When position_error < 0.05m, advances to waypoint 1 (x=1.5)"
echo "  3. When position_error < 0.05m, advances to waypoint 2 (x=2.0)"
echo "  4. When final waypoint reached + is_final_segment=true:"
echo "     → Sets trajectory_complete_ = true"
echo "     → Publishes zero velocity (chassis stops)"
echo "     → Logs: 'Published zero velocity - chassis stopped'"
echo ""
echo "Monitor the node's terminal output for waypoint advancement messages!"
echo ""
echo -e "${BLUE}Press Ctrl+C to stop${NC}"

# Keep script running
wait $JOINT_PUB_PID
