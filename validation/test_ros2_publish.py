#!/usr/bin/env python3
"""
Simple ROS2 publisher test to verify publishing works in WSL
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import Header

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')
        
        # Create publishers
        self.arm_pub = self.create_publisher(JointState, '/hdas/feedback_arm_left', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom_wheel', 10)
        
        # Create timer to publish at 10 Hz
        self.timer = self.create_timer(0.1, self.publish_state)
        self.count = 0
        
        self.get_logger().info('Simple publisher started')
    
    def publish_state(self):
        self.count += 1
        
        # Publish JointState
        joint_msg = JointState()
        joint_msg.header = Header()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.header.frame_id = 'base_link'
        joint_msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        joint_msg.position = [0.0] * 6
        joint_msg.velocity = [0.0] * 6
        joint_msg.effort = [0.0] * 6
        self.arm_pub.publish(joint_msg)
        
        # Publish Odometry
        odom_msg = Odometry()
        odom_msg.header = Header()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        self.odom_pub.publish(odom_msg)
        
        if self.count % 10 == 0:
            self.get_logger().info(f'Published {self.count} messages')

def main():
    rclpy.init()
    node = SimplePublisher()
    
    try:
        print('\n' + '='*60)
        print('Simple ROS2 Publisher Test')
        print('='*60)
        print('\nPublishing to:')
        print('  - /hdas/feedback_arm_left (JointState)')
        print('  - /odom_wheel (Odometry)')
        print('\nPress Ctrl+C to stop\n')
        
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\n\nStopped by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
