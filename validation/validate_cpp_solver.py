#!/usr/bin/env python3
"""
Validate C++ Solver Against MATLAB Implementation
This script runs the same trajectory through the C++ solver via ROS2
and saves results for comparison with MATLAB.

Author: Generated for gikWBC9DOF validation
Date: 2025-10-06
"""

import json
import time
import numpy as np
from pathlib import Path

import rclpy
from rclpy.node import Node
from gik9dof_msgs.msg import TargetTrajectory, SolverDiagnostics
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry

class SolverValidator(Node):
    def __init__(self):
        super().__init__('solver_validator')
        
        # Configuration
        self.trajectory_file = Path('../1_pull_world_scaled.json')
        self.output_file = Path('../validation_results_cpp.json')
        
        # Publishers
        self.traj_pub = self.create_publisher(
            TargetTrajectory,
            '/gik9dof/target_trajectory',
            10
        )
        
        self.arm_pub = self.create_publisher(
            JointState,
            '/hdas/feedback_arm_left',
            10
        )
        
        self.odom_pub = self.create_publisher(
            Odometry,
            '/odom_wheel',
            10
        )
        
        # Subscribers
        self.diag_sub = self.create_subscription(
            SolverDiagnostics,
            '/gik9dof/diagnostics',
            self.diagnostics_callback,
            10
        )
        
        # State
        self.results = []
        self.current_waypoint_idx = 0
        self.waiting_for_result = False
        self.trajectory = []
        self.start_time = None
        
        self.get_logger().info('Solver Validator initialized')
    
    def load_trajectory(self):
        """Load trajectory from JSON file"""
        self.get_logger().info(f'Loading trajectory from {self.trajectory_file}...')
        
        with open(self.trajectory_file, 'r') as f:
            data = json.load(f)
        
        self.trajectory = data['poses']
        self.get_logger().info(f'Loaded {len(self.trajectory)} waypoints')
    
    def diagnostics_callback(self, msg):
        """Handle solver diagnostics"""
        if not self.waiting_for_result:
            return
        
        # Store result
        result = {
            'index': self.current_waypoint_idx + 1,
            'configuration': list(msg.joint_positions),
            'solve_time_ms': msg.solve_time * 1000.0,
            'iterations': msg.iterations,
            'status': msg.status,
            'target_position': self.trajectory[self.current_waypoint_idx]['position'],
            'target_orientation': self.trajectory[self.current_waypoint_idx]['orientation']
        }
        
        self.results.append(result)
        self.waiting_for_result = False
        
        # Progress
        progress = (self.current_waypoint_idx / (len(self.trajectory) - 1)) * 100
        self.get_logger().info(
            f'Progress: {progress:.0f}% (waypoint {self.current_waypoint_idx+1}/{len(self.trajectory)}), '
            f'Status: {msg.status}, Time: {msg.solve_time*1000:.2f} ms'
        )
    
    def publish_robot_state(self, config):
        """Publish current robot state"""
        # Publish arm state
        arm_msg = JointState()
        arm_msg.header.stamp = self.get_clock().now().to_msg()
        arm_msg.name = [f'left_arm_joint{i+1}' for i in range(6)]
        arm_msg.position = list(config[3:9])  # Last 6 joints are arm
        self.arm_pub.publish(arm_msg)
        
        # Publish base odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = arm_msg.header.stamp
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.position.x = config[0]
        odom_msg.pose.pose.position.y = config[1]
        odom_msg.pose.pose.position.z = 0.0
        
        # Convert theta to quaternion (rotation around z-axis)
        theta = config[2]
        odom_msg.pose.pose.orientation.w = np.cos(theta / 2)
        odom_msg.pose.pose.orientation.z = np.sin(theta / 2)
        
        self.odom_pub.publish(odom_msg)
    
    def publish_trajectory_point(self, idx):
        """Publish target trajectory waypoint"""
        pose = self.trajectory[idx]
        
        msg = TargetTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        
        # Only send single waypoint (current target)
        msg.positions = [pose['position']]
        msg.orientations = [pose['orientation']]
        
        self.traj_pub.publish(msg)
    
    def run_validation(self):
        """Main validation loop"""
        self.load_trajectory()
        
        # Wait for node to be ready
        time.sleep(1.0)
        
        # Initialize with first waypoint (assume already at this position)
        self.get_logger().info('Solving trajectory...')
        
        initial_config = [
            self.trajectory[0]['position'][0],  # base_x
            self.trajectory[0]['position'][1],  # base_y
            0.0,                                 # base_theta
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0        # arm joints (will be solved)
        ]
        
        current_config = initial_config.copy()
        
        # Process each waypoint
        for idx in range(len(self.trajectory)):
            self.current_waypoint_idx = idx
            self.waiting_for_result = True
            
            # Publish current state
            self.publish_robot_state(current_config)
            time.sleep(0.01)  # Small delay for message delivery
            
            # Publish target
            self.publish_trajectory_point(idx)
            
            # Wait for result (with timeout)
            timeout = 5.0  # 5 seconds
            start = time.time()
            while self.waiting_for_result and (time.time() - start) < timeout:
                rclpy.spin_once(self, timeout_sec=0.01)
            
            if self.waiting_for_result:
                self.get_logger().error(f'Timeout waiting for result at waypoint {idx+1}')
                break
            
            # Update current config from result
            if self.results:
                current_config = self.results[-1]['configuration']
        
        # Save results
        self.save_results()
    
    def save_results(self):
        """Save validation results to JSON"""
        self.get_logger().info(f'Saving results to {self.output_file}...')
        
        # Calculate statistics
        solve_times = [r['solve_time_ms'] for r in self.results if r['index'] > 1]
        success_count = sum(1 for r in self.results if r['status'] == 'success' and r['index'] > 1)
        
        output = {
            'metadata': {
                'solver': 'C++ (via ROS2)',
                'num_waypoints': len(self.trajectory),
                'success_rate': (success_count / (len(self.results) - 1) * 100) if len(self.results) > 1 else 0,
                'avg_solve_time_ms': np.mean(solve_times) if solve_times else 0,
                'max_solve_time_ms': max(solve_times) if solve_times else 0
            },
            'waypoints': self.results
        }
        
        with open(self.output_file, 'w') as f:
            json.dump(output, f, indent=2)
        
        self.get_logger().info('Results saved!')
        self.get_logger().info(f'  Success rate: {output["metadata"]["success_rate"]:.1f}%')
        self.get_logger().info(f'  Avg solve time: {output["metadata"]["avg_solve_time_ms"]:.2f} ms')

def main(args=None):
    rclpy.init(args=args)
    
    validator = SolverValidator()
    
    try:
        validator.run_validation()
    except KeyboardInterrupt:
        pass
    finally:
        validator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
