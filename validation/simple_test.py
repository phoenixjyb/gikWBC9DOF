#!/usr/bin/env python3
"""
Simple validation test - publishes test data to gik9dof_solver node
and monitors diagnostics output.
"""

import json
import time
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from gik9dof_msgs.msg import EndEffectorTrajectory, SolverDiagnostics
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped

class SimpleValidator(Node):
    def __init__(self):
        super().__init__('simple_validator')
        
        # Publishers - use default QoS to match solver expectations
        self.traj_pub = self.create_publisher(
            EndEffectorTrajectory,
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
        
        # Subscriber
        self.diag_sub = self.create_subscription(
            SolverDiagnostics,
            '/gik9dof/diagnostics',
            self.diagnostics_callback,
            10
        )
        
        self.diag_received = False
        self.last_diag = None
        
        self.get_logger().info('Simple Validator initialized')
    
    def diagnostics_callback(self, msg):
        """Handle solver diagnostics"""
        self.diag_received = True
        self.last_diag = msg
        
        self.get_logger().info(
            f'Diagnostics: status={msg.status}, '
            f'time={msg.solve_time*1000:.2f}ms, '
            f'iterations={msg.iterations}'
        )
    
    def publish_robot_state(self, base_x=0.0, base_y=0.0, base_theta=0.0):
        """Publish current robot state"""
        # Publish arm state (6 joints at zero)
        arm_msg = JointState()
        arm_msg.header.stamp = self.get_clock().now().to_msg()
        arm_msg.name = [f'left_arm_joint{i+1}' for i in range(6)]
        arm_msg.position = [0.0] * 6
        self.arm_pub.publish(arm_msg)
        
        # Publish base odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = arm_msg.header.stamp
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.position.x = base_x
        odom_msg.pose.pose.position.y = base_y
        odom_msg.pose.pose.position.z = 0.0
        
        # Convert theta to quaternion
        import math
        odom_msg.pose.pose.orientation.w = math.cos(base_theta / 2)
        odom_msg.pose.pose.orientation.z = math.sin(base_theta / 2)
        
        self.odom_pub.publish(odom_msg)
    
    def publish_test_trajectory(self):
        """Publish a simple test trajectory"""
        msg = EndEffectorTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        
        # Single test waypoint
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = msg.header.stamp
        pose_stamped.header.frame_id = 'world'
        pose_stamped.pose.position = Point(x=1.65, y=0.08, z=0.86)
        pose_stamped.pose.orientation = Quaternion(x=-0.5, y=0.5, z=-0.5, w=0.5)
        
        msg.waypoints = [pose_stamped]
        msg.timestamps = [0.0]
        msg.sequence_id = 1
        msg.is_final_segment = True
        msg.lookahead_time = 1.0
        msg.max_velocity = 0.5
        msg.max_acceleration = 1.0
        
        self.traj_pub.publish(msg)
        self.get_logger().info('Published test trajectory')
    
    def run_test(self):
        """Run simple validation test"""
        self.get_logger().info('Starting validation test...')
        
        # Wait MUCH longer for DDS discovery to complete
        self.get_logger().info('Waiting for DDS discovery (5 seconds)...')
        time.sleep(5.0)
        
        # Publish robot state messages slowly
        self.get_logger().info('Publishing robot state (slowly, 5 times)...')
        for i in range(5):
            self.get_logger().info(f'  Publishing state #{i+1}...')
            self.publish_robot_state(base_x=1.65, base_y=0.08, base_theta=0.0)
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(1.0)  # 1 second between publishes
        
        # Give solver time to receive
        self.get_logger().info('Waiting for solver (3 seconds)...')
        time.sleep(3.0)
        
        # Publish test trajectory
        self.get_logger().info('Publishing test trajectory...')
        for i in range(3):
            self.publish_test_trajectory()
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(1.0)
        
        # Wait for diagnostics
        self.get_logger().info('Waiting for solver diagnostics...')
        timeout = 10.0
        start = time.time()
        while not self.diag_received and (time.time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if self.diag_received:
            self.get_logger().info('✅ Test PASSED - Solver responded!')
            self.get_logger().info(f'   Status: {self.last_diag.status}')
            self.get_logger().info(f'   Solve time: {self.last_diag.solve_time*1000:.2f} ms')
            self.get_logger().info(f'   Iterations: {self.last_diag.iterations}')
            return True
        else:
            self.get_logger().error('❌ Test FAILED - No diagnostics received')
            return False

def main(args=None):
    rclpy.init(args=args)
    
    validator = SimpleValidator()
    
    try:
        success = validator.run_test()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        pass
    finally:
        validator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
