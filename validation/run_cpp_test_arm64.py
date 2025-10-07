#!/usr/bin/env python3
"""
C++ ARM64 Solver Test Script
=============================
Tests the ARM64-compiled C++ IK solver by sending waypoints and collecting results.

Author: Generated for ARM64 validation
Date: 2025-10-07
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from gik9dof_msgs.msg import SolverDiagnostics, EndEffectorTrajectory
from std_msgs.msg import Header
import json
import time
import sys
from pathlib import Path


class CPPSolverTester(Node):
    """Test node for validating C++ ARM64 solver"""
    
    def __init__(self):
        super().__init__('cpp_solver_tester')
        
        # Publishers for robot state (solver needs these!)
        self.arm_feedback_pub = self.create_publisher(
            JointState,
            '/hdas/feedback_arm_left',
            10
        )
        
        self.odom_pub = self.create_publisher(
            Odometry,
            '/odom_wheel',
            10
        )
        
        # Publishers for target trajectory
        self.trajectory_pub = self.create_publisher(
            EndEffectorTrajectory,
            '/gik9dof/target_trajectory',
            10
        )
        
        # Subscribers
        self.joint_sub = self.create_subscription(
            JointState,
            '/motion_target/target_joint_state_arm_left',
            self.joint_callback,
            10
        )
        
        self.diag_sub = self.create_subscription(
            SolverDiagnostics,
            '/gik9dof/solver_diagnostics',
            self.diagnostics_callback,
            10
        )
        
        # State tracking
        self.latest_joint_state = None
        self.latest_diagnostics = None
        self.waiting_for_response = False
        
        self.get_logger().info('C++ ARM64 Solver Tester initialized')
    
    def joint_callback(self, msg):
        """Callback for joint state messages"""
        self.latest_joint_state = msg
    
    def diagnostics_callback(self, msg):
        """Callback for solver diagnostics"""
        self.latest_diagnostics = msg
        if self.waiting_for_response:
            self.waiting_for_response = False
    
    def publish_robot_state(self, joint_config=None):
        """Publish current robot state (arm joints + base odom)"""
        # Default to home configuration if not provided
        if joint_config is None:
            joint_config = [0.0, 0.0, 0.0,  # base: x, y, theta
                          0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # arm: 6 joints
        
        # Publish arm joint states (last 6 joints)
        arm_msg = JointState()
        arm_msg.header.stamp = self.get_clock().now().to_msg()
        arm_msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        arm_msg.position = joint_config[3:9]  # Last 6 values
        self.arm_feedback_pub.publish(arm_msg)
        
        # Publish base odometry (first 3 joints: x, y, theta)
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        
        odom_msg.pose.pose.position.x = joint_config[0]
        odom_msg.pose.pose.position.y = joint_config[1]
        odom_msg.pose.pose.position.z = 0.0
        
        # Convert theta to quaternion (rotation around z-axis)
        import math
        theta = joint_config[2]
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = math.sin(theta / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(theta / 2.0)
        
        self.odom_pub.publish(odom_msg)
    
    def send_target_pose(self, position, orientation, sequence_id=0):
        """Send a target pose to the solver as a single-waypoint trajectory"""
        # Create a single waypoint
        waypoint = PoseStamped()
        waypoint.header.stamp = self.get_clock().now().to_msg()
        waypoint.header.frame_id = 'world'
        
        waypoint.pose.position.x = float(position[0])
        waypoint.pose.position.y = float(position[1])
        waypoint.pose.position.z = float(position[2])
        
        waypoint.pose.orientation.x = float(orientation[0])
        waypoint.pose.orientation.y = float(orientation[1])
        waypoint.pose.orientation.z = float(orientation[2])
        waypoint.pose.orientation.w = float(orientation[3])
        
        # Create trajectory message with single waypoint
        traj_msg = EndEffectorTrajectory()
        traj_msg.header.stamp = self.get_clock().now().to_msg()
        traj_msg.header.frame_id = 'world'
        traj_msg.waypoints = [waypoint]
        traj_msg.timestamps = [0.0]  # Immediate execution
        traj_msg.sequence_id = sequence_id
        traj_msg.is_final_segment = True
        traj_msg.lookahead_time = 0.0
        traj_msg.max_velocity = 1.0
        traj_msg.max_acceleration = 1.0
        
        self.trajectory_pub.publish(traj_msg)
        self.waiting_for_response = True
    
    def wait_for_solution(self, timeout_sec=2.0):
        """Wait for solver to respond"""
        start_time = time.time()
        rate = self.create_rate(100)  # 100 Hz
        
        while self.waiting_for_response and (time.time() - start_time) < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.01)
            rate.sleep()
        
        return not self.waiting_for_response


def main():
    print("="*60)
    print("C++ ARM64 Solver Validation Test")
    print("="*60)
    print()
    
    # Load MATLAB reference data
    ref_file = Path.home() / 'gikWBC9DOF' / 'validation' / 'matlab_reference_results.json'
    
    if not ref_file.exists():
        print(f"❌ ERROR: Reference file not found: {ref_file}")
        print("Please ensure matlab_reference_results.json is in ~/gikWBC9DOF/validation/")
        return 1
    
    print(f"Step 1: Loading MATLAB reference from {ref_file}")
    with open(ref_file, 'r') as f:
        ref_data = json.load(f)
    
    num_waypoints = ref_data['metadata']['num_waypoints']
    print(f"  ✅ Loaded {num_waypoints} waypoints")
    print()
    
    # Initialize ROS2
    print("Step 2: Initializing ROS2 node...")
    rclpy.init()
    tester = CPPSolverTester()
    
    # Wait for solver node to be ready
    print("Step 3: Waiting for solver node...")
    print("  (Make sure gik9dof_solver_node is running)")
    
    # Publish initial robot state to let solver initialize
    print("  Publishing initial robot state...")
    initial_config = [0.0] * 9  # Home configuration
    for _ in range(10):  # Publish multiple times to ensure receipt
        tester.publish_robot_state(initial_config)
        time.sleep(0.1)
    
    print("  ✅ Initial state published")
    print()
    
    # Test each waypoint
    print(f"Step 4: Testing {num_waypoints} waypoints...")
    print("  Progress: ", end='', flush=True)
    
    results = {
        'metadata': {
            'platform': 'ARM64 Ubuntu 22.04',
            'architecture': 'aarch64',
            'date': time.strftime('%Y-%m-%d %H:%M:%S'),
            'num_waypoints': num_waypoints,
            'reference_file': str(ref_file)
        },
        'waypoints': []
    }
    
    success_count = 0
    current_config = initial_config  # Start from home config
    
    for i, wp in enumerate(ref_data['waypoints'], 1):
        # Show progress
        if i % 5 == 0 or i == 1:
            print(f"{i}", end='', flush=True)
        else:
            print('.', end='', flush=True)
        
        # Publish current robot state (solver needs to know where robot is)
        tester.publish_robot_state(current_config)
        time.sleep(0.05)  # Small delay for message delivery
        
        # Send target pose
        position = wp['target_position']
        orientation = wp['target_orientation']
        
        tester.send_target_pose(position, orientation, sequence_id=i)
        
        # Wait for solution
        start_time = time.time()
        got_response = tester.wait_for_solution(timeout_sec=2.0)
        solve_time_ms = (time.time() - start_time) * 1000
        
        # Collect results
        waypoint_result = {
            'index': wp['index'],
            'target_position': position,
            'target_orientation': orientation,
            'joint_config': None,
            'solve_time_ms': solve_time_ms,
            'status': 'timeout',
            'iterations': 0,
            'pose_error': 0.0
        }
        
        if got_response and tester.latest_joint_state:
            waypoint_result['joint_config'] = list(tester.latest_joint_state.position)
            waypoint_result['status'] = 'success'
            success_count += 1
            
            # Update current config for next iteration (use solved config)
            if len(tester.latest_joint_state.position) >= 6:
                # Combine base (previous) + arm (new solution)
                current_config = current_config[:3] + list(tester.latest_joint_state.position[:6])
            
            if tester.latest_diagnostics:
                waypoint_result['iterations'] = tester.latest_diagnostics.iterations
                waypoint_result['pose_error'] = tester.latest_diagnostics.pose_error
        
        results['waypoints'].append(waypoint_result)
        
        # Small delay between waypoints
        time.sleep(0.1)
    
    print("  DONE")
    print()
    
    # Compute summary statistics
    print("Step 5: Computing summary statistics...")
    
    solve_times = [wp['solve_time_ms'] for wp in results['waypoints']]
    iterations = [wp['iterations'] for wp in results['waypoints'] if wp['iterations'] > 0]
    
    results['summary'] = {
        'total_waypoints': num_waypoints,
        'success_count': success_count,
        'success_rate': success_count / num_waypoints,
        'avg_solve_time_ms': sum(solve_times) / len(solve_times) if solve_times else 0,
        'max_solve_time_ms': max(solve_times) if solve_times else 0,
        'min_solve_time_ms': min(solve_times) if solve_times else 0,
        'avg_iterations': sum(iterations) / len(iterations) if iterations else 0
    }
    
    print(f"  Success rate: {success_count}/{num_waypoints} ({results['summary']['success_rate']*100:.1f}%)")
    print(f"  Solve time: {results['summary']['avg_solve_time_ms']:.2f} ms (avg)")
    print()
    
    # Save results
    output_file = Path.home() / 'gikWBC9DOF' / 'validation' / 'cpp_arm64_results.json'
    print(f"Step 6: Saving results to {output_file}")
    
    with open(output_file, 'w') as f:
        json.dump(results, f, indent=2)
    
    file_size_kb = output_file.stat().st_size / 1024
    print(f"  ✅ Results saved ({file_size_kb:.1f} KB)")
    print()
    
    # Cleanup
    tester.destroy_node()
    rclpy.shutdown()
    
    print("="*60)
    print("✅ C++ ARM64 Solver Test Complete")
    print("="*60)
    print()
    print("Next steps:")
    print("  1. Copy cpp_arm64_results.json back to Windows")
    print("  2. Run comparison analysis (Phase 4)")
    print()
    
    return 0


if __name__ == '__main__':
    sys.exit(main())
