#!/usr/bin/env python3
"""
ROS2 Hybrid A* Planner Service Node (Python + MATLAB Engine)

This node provides path planning services using the MATLAB Hybrid A* implementation.
It's much simpler than C++ codegen and easier to integrate with existing ROS2 nodes.

Services:
    /plan_path (nav_msgs/srv/GetPlan) - Plan path from start to goal

Publishers:
    /planned_path (nav_msgs/msg/Path) - Planned path visualization
    
Subscribers:
    /occupancy_grid (nav_msgs/msg/OccupancyGrid) - Environment map
    /goal_pose (geometry_msgs/msg/PoseStamped) - Goal position
    /initialpose (geometry_msgs/msg/PoseWithCovarianceStamped) - Start position

Author: WHEELTEC Mobile Manipulator Project
Date: 2025-10-07
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion
from nav_msgs.srv import GetPlan

import numpy as np
import matlab.engine
from pathlib import Path as PathLib
import tf_transformations
import time


class HybridAStarPlannerNode(Node):
    """ROS2 node that provides Hybrid A* planning via MATLAB Engine"""
    
    def __init__(self):
        super().__init__('hybrid_astar_planner')
        
        # Parameters
        self.declare_parameter('matlab_workspace', 
                              str(PathLib(__file__).parent.parent.parent / 'matlab'))
        self.declare_parameter('planning_timeout', 5.0)
        self.declare_parameter('inflation_radius', 0.511)
        self.declare_parameter('grid_resolution', 0.1)
        
        self.matlab_workspace = self.get_parameter('matlab_workspace').value
        self.planning_timeout = self.get_parameter('planning_timeout').value
        self.inflation_radius = self.get_parameter('inflation_radius').value
        self.grid_resolution = self.get_parameter('grid_resolution').value
        
        # Initialize MATLAB Engine
        self.get_logger().info('Starting MATLAB Engine...')
        try:
            self.matlab = matlab.engine.start_matlab()
            self.matlab.addpath(self.matlab.genpath(self.matlab_workspace), nargout=0)
            self.get_logger().info(f'✓ MATLAB Engine started with workspace: {self.matlab_workspace}')
        except Exception as e:
            self.get_logger().error(f'Failed to start MATLAB Engine: {e}')
            raise
        
        # QoS profiles
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Publishers
        self.path_pub = self.create_publisher(Path, '/planned_path', qos_reliable)
        
        # Subscribers
        self.grid_sub = self.create_subscription(
            OccupancyGrid,
            '/occupancy_grid',
            self.grid_callback,
            qos_reliable
        )
        
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            qos_reliable
        )
        
        self.start_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.start_callback,
            qos_reliable
        )
        
        # Service
        self.plan_service = self.create_service(
            GetPlan,
            '/plan_path',
            self.plan_path_service
        )
        
        # State
        self.current_grid = None
        self.start_pose = None
        self.goal_pose = None
        self.grid_received = False
        
        self.get_logger().info('Hybrid A* Planner Node initialized')
        self.get_logger().info('Subscribed to: /occupancy_grid, /goal_pose, /initialpose')
        self.get_logger().info('Service available: /plan_path')
        self.get_logger().info('Publishing to: /planned_path')
    
    def grid_callback(self, msg):
        """Store occupancy grid"""
        self.current_grid = msg
        self.grid_received = True
        self.get_logger().debug(
            f'Received grid: {msg.info.width}×{msg.info.height} @ {msg.info.resolution}m'
        )
    
    def start_callback(self, msg):
        """Store start pose from /initialpose"""
        self.start_pose = PoseStamped()
        self.start_pose.header = msg.header
        self.start_pose.pose = msg.pose.pose
        
        yaw = self.quaternion_to_yaw(self.start_pose.pose.orientation)
        self.get_logger().info(
            f'Start pose updated: ({self.start_pose.pose.position.x:.2f}, '
            f'{self.start_pose.pose.position.y:.2f}, {yaw:.2f})'
        )
    
    def goal_callback(self, msg):
        """Store goal pose and trigger planning"""
        self.goal_pose = msg
        
        yaw = self.quaternion_to_yaw(self.goal_pose.pose.orientation)
        self.get_logger().info(
            f'Goal pose received: ({self.goal_pose.pose.position.x:.2f}, '
            f'{self.goal_pose.pose.position.y:.2f}, {yaw:.2f})'
        )
        
        # Auto-trigger planning if we have all inputs
        if self.start_pose and self.grid_received:
            path = self.plan_path()
            if path:
                self.path_pub.publish(path)
        else:
            if not self.start_pose:
                self.get_logger().warn('Waiting for start pose (/initialpose)')
            if not self.grid_received:
                self.get_logger().warn('Waiting for occupancy grid')
    
    def plan_path_service(self, request, response):
        """Service callback for planning"""
        self.get_logger().info('Planning service called')
        
        # Use service request or stored poses
        start = request.start if hasattr(request, 'start') and request.start.header.frame_id else self.start_pose
        goal = request.goal if hasattr(request, 'goal') and request.goal.header.frame_id else self.goal_pose
        
        if not start or not goal or not self.grid_received:
            self.get_logger().error('Missing start/goal/grid for planning')
            response.plan = Path()
            return response
        
        # Override stored poses with service request
        old_start, old_goal = self.start_pose, self.goal_pose
        if hasattr(request, 'start') and request.start.header.frame_id:
            self.start_pose = request.start
        if hasattr(request, 'goal') and request.goal.header.frame_id:
            self.goal_pose = request.goal
        
        path = self.plan_path()
        response.plan = path if path else Path()
        
        # Restore
        self.start_pose, self.goal_pose = old_start, old_goal
        
        return response
    
    def plan_path(self):
        """Call MATLAB Hybrid A* planner and convert result to ROS Path"""
        if not all([self.start_pose, self.goal_pose, self.grid_received]):
            self.get_logger().error('Cannot plan: missing start/goal/grid')
            return None
        
        start_time = time.time()
        
        try:
            # Extract poses
            start_x = self.start_pose.pose.position.x
            start_y = self.start_pose.pose.position.y
            start_theta = self.quaternion_to_yaw(self.start_pose.pose.orientation)
            
            goal_x = self.goal_pose.pose.position.x
            goal_y = self.goal_pose.pose.position.y
            goal_theta = self.quaternion_to_yaw(self.goal_pose.pose.orientation)
            
            self.get_logger().info(
                f'Planning from ({start_x:.2f}, {start_y:.2f}, {start_theta:.2f}) '
                f'to ({goal_x:.2f}, {goal_y:.2f}, {goal_theta:.2f})'
            )
            
            # Convert occupancy grid to MATLAB format
            grid_matlab = self.ros_grid_to_matlab(self.current_grid)
            
            # Create MATLAB HybridState objects
            start_state = self.matlab.gik9dof.HybridState()
            start_state.x = float(start_x)
            start_state.y = float(start_y)
            start_state.theta = float(start_theta)
            
            goal_state = self.matlab.gik9dof.HybridState()
            goal_state.x = float(goal_x)
            goal_state.y = float(goal_y)
            goal_state.theta = float(goal_theta)
            
            # Call MATLAB planner (use codegen version for speed)
            self.get_logger().info('Calling MATLAB planner...')
            path_matlab, stats = self.matlab.gik9dof.planHybridAStarCodegen(
                start_state, goal_state, grid_matlab, nargout=2
            )
            
            plan_time = time.time() - start_time
            
            # Check success
            if not bool(stats['success']):
                self.get_logger().warn(f'Planning failed after {plan_time:.3f}s')
                return None
            
            # Convert MATLAB path to ROS Path
            ros_path = self.matlab_path_to_ros(path_matlab, stats)
            
            self.get_logger().info(
                f'✓ Path found: {stats["path_length"]} waypoints, '
                f'{stats["path_cost"]:.2f}m, {plan_time:.3f}s'
            )
            
            return ros_path
            
        except Exception as e:
            self.get_logger().error(f'Planning failed: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())
            return None
    
    def ros_grid_to_matlab(self, grid_msg):
        """Convert ROS OccupancyGrid to MATLAB OccupancyGrid2D"""
        # Extract grid data
        width = grid_msg.info.width
        height = grid_msg.info.height
        resolution = grid_msg.info.resolution
        origin_x = grid_msg.info.origin.position.x
        origin_y = grid_msg.info.origin.position.y
        
        # Convert data to numpy array (ROS uses row-major, MATLAB uses column-major)
        data = np.array(grid_msg.data, dtype=np.int8).reshape((height, width))
        
        # Convert to binary (unknown=-1, free=0, occupied=1-100 → binary 0/1)
        # Treat unknown as free for now
        binary_grid = (data > 50).astype(bool)
        
        # Create MATLAB grid object
        grid = self.matlab.gik9dof.OccupancyGrid2D(
            resolution, int(width), int(height), origin_x, origin_y
        )
        
        # Set data (convert numpy bool to MATLAB logical)
        grid.data = matlab.logical(binary_grid.tolist())
        
        # Inflate obstacles
        params = self.matlab.gik9dof.getChassisParams()
        grid = self.matlab.gik9dof.inflateObstacles(grid, params.inflation_radius)
        
        return grid
    
    def matlab_path_to_ros(self, path_matlab, stats):
        """Convert MATLAB waypoint struct array to ROS Path"""
        ros_path = Path()
        ros_path.header.stamp = self.get_clock().now().to_msg()
        ros_path.header.frame_id = self.goal_pose.header.frame_id
        
        # Extract waypoints from MATLAB struct
        path_length = int(stats['path_length'])
        
        for i in range(path_length):
            waypoint = path_matlab[i]
            
            pose = PoseStamped()
            pose.header = ros_path.header
            pose.pose.position.x = float(waypoint['x'])
            pose.pose.position.y = float(waypoint['y'])
            pose.pose.position.z = 0.0
            
            # Convert theta to quaternion
            quat = self.yaw_to_quaternion(float(waypoint['theta']))
            pose.pose.orientation = quat
            
            ros_path.poses.append(pose)
        
        return ros_path
    
    @staticmethod
    def quaternion_to_yaw(q):
        """Extract yaw from quaternion"""
        return tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
    
    @staticmethod
    def yaw_to_quaternion(yaw):
        """Convert yaw to quaternion"""
        quat_list = tf_transformations.quaternion_from_euler(0, 0, yaw)
        q = Quaternion()
        q.x, q.y, q.z, q.w = quat_list
        return q
    
    def destroy_node(self):
        """Cleanup MATLAB engine"""
        if hasattr(self, 'matlab'):
            self.get_logger().info('Stopping MATLAB Engine...')
            self.matlab.quit()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = HybridAStarPlannerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Fatal error: {e}')
        import traceback
        traceback.print_exc()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
