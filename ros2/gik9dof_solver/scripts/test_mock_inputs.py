#!/usr/bin/env python3
"""
test_mock_inputs.py
Publishes mock robot state for testing gik9dof_solver_node
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from gik9dof_msgs.msg import EndEffectorTrajectory
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
import math


class MockInputPublisher(Node):
    def __init__(self):
        super().__init__('mock_input_publisher')
        
        # Publishers
        self.joint_pub = self.create_publisher(
            JointState, '/hdas/feedback_arm_left', 10)
        self.odom_pub = self.create_publisher(
            Odometry, '/odom_wheel', 10)
        self.traj_pub = self.create_publisher(
            EndEffectorTrajectory, '/gik9dof/target_trajectory', 10)
        
        # Timer for periodic publishing
        self.timer = self.create_timer(0.1, self.publish_mock_data)  # 10 Hz
        
        # State variables
        self.t = 0.0
        self.trajectory_sent = False
        
        self.get_logger().info('Mock input publisher started')
        self.get_logger().info('Publishing on:')
        self.get_logger().info('  - /hdas/feedback_arm_left (JointState)')
        self.get_logger().info('  - /odom_wheel (Odometry)')
        self.get_logger().info('  - /gik9dof/target_trajectory (EndEffectorTrajectory)')
    
    def publish_mock_data(self):
        self.t += 0.1
        
        # Publish joint state (6 arm joints)
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.header.frame_id = 'base'
        js.name = [
            'left_arm_joint1', 'left_arm_joint2', 'left_arm_joint3',
            'left_arm_joint4', 'left_arm_joint5', 'left_arm_joint6'
        ]
        # Simple sinusoidal motion
        js.position = [
            0.0,
            0.5 + 0.1 * math.sin(self.t),
            -0.3,
            0.0,
            0.2 + 0.1 * math.cos(self.t),
            0.0
        ]
        js.velocity = [0.0] * 6
        js.effort = [0.0] * 6
        self.joint_pub.publish(js)
        
        # Publish odometry (base: x, y, theta)
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        # Slowly moving forward
        odom.pose.pose.position.x = 1.0 + 0.05 * self.t
        odom.pose.pose.position.y = 0.5
        odom.pose.pose.position.z = 0.0
        
        # Slowly rotating
        theta = 0.1 + 0.02 * self.t
        odom.pose.pose.orientation.w = math.cos(theta / 2.0)
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(theta / 2.0)
        
        odom.twist.twist.linear.x = 0.05
        odom.twist.twist.angular.z = 0.02
        
        self.odom_pub.publish(odom)
        
        # Publish trajectory (once every 5 seconds)
        if not self.trajectory_sent or (self.t % 5.0 < 0.15):
            self.publish_trajectory()
            self.trajectory_sent = True
    
    def publish_trajectory(self):
        traj = EndEffectorTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.header.frame_id = 'odom'
        
        # Create a simple trajectory with 5 waypoints
        waypoints = [
            ([0.6, 0.0, 0.9], [0.0, 0.0, 0.0, 1.0]),  # Straight ahead, up
            ([0.6, 0.1, 0.9], [0.0, 0.0, 0.0, 1.0]),  # Slightly right
            ([0.6, 0.2, 0.9], [0.0, 0.0, 0.0, 1.0]),  # More right
            ([0.6, 0.1, 0.9], [0.0, 0.0, 0.0, 1.0]),  # Back to slight right
            ([0.6, 0.0, 0.9], [0.0, 0.0, 0.0, 1.0]),  # Back to center
        ]
        
        for i, (pos, quat) in enumerate(waypoints):
            ps = PoseStamped()
            ps.header = traj.header
            ps.pose.position.x = pos[0]
            ps.pose.position.y = pos[1]
            ps.pose.position.z = pos[2]
            ps.pose.orientation.x = quat[0]
            ps.pose.orientation.y = quat[1]
            ps.pose.orientation.z = quat[2]
            ps.pose.orientation.w = quat[3]
            traj.waypoints.append(ps)
        
        # Timestamps (0.5s between waypoints)
        traj.timestamps = [i * 0.5 for i in range(len(waypoints))]
        
        # Metadata
        traj.sequence_id = int(self.t / 5.0)
        traj.is_final_segment = False
        traj.lookahead_time = traj.timestamps[-1]
        traj.max_velocity = 0.2
        traj.max_acceleration = 0.5
        
        self.traj_pub.publish(traj)
        self.get_logger().info(f'Published trajectory with {len(waypoints)} waypoints (seq={traj.sequence_id})')


def main(args=None):
    rclpy.init(args=args)
    node = MockInputPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
