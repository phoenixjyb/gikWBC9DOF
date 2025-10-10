"""
chassis_path_follower_launch.py

Launch file for the chassis path follower node.
Starts the node with configurable parameters for all 3 controller modes.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for chassis path follower."""
    
    # Declare launch arguments
    controller_mode_arg = DeclareLaunchArgument(
        'controller_mode',
        default_value='2',
        description='Controller mode: 0=differentiation, 1=heading, 2=pure pursuit'
    )
    
    # Velocity limits
    vx_max_arg = DeclareLaunchArgument('vx_max', default_value='1.0')
    vx_min_arg = DeclareLaunchArgument('vx_min', default_value='-0.5')
    wz_max_arg = DeclareLaunchArgument('wz_max', default_value='1.0')
    
    # Acceleration limits
    accel_max_arg = DeclareLaunchArgument('accel_max', default_value='0.5')
    decel_max_arg = DeclareLaunchArgument('decel_max', default_value='0.8')
    jerk_limit_arg = DeclareLaunchArgument('jerk_limit', default_value='2.0')
    
    # Lookahead parameters
    lookahead_base_arg = DeclareLaunchArgument('lookahead_base', default_value='0.3')
    lookahead_gain_arg = DeclareLaunchArgument('lookahead_gain', default_value='0.5')
    lookahead_min_arg = DeclareLaunchArgument('lookahead_min', default_value='0.2')
    lookahead_max_arg = DeclareLaunchArgument('lookahead_max', default_value='2.0')
    
    # Pure pursuit parameters (Mode 2)
    kappa_threshold_arg = DeclareLaunchArgument('kappa_threshold', default_value='0.5')
    vx_reduction_arg = DeclareLaunchArgument('vx_reduction', default_value='0.3')
    
    # Heading controller gains (Mode 1)
    heading_kp_arg = DeclareLaunchArgument('heading_kp', default_value='2.0')
    feedforward_gain_arg = DeclareLaunchArgument('feedforward_gain', default_value='0.8')
    
    # Chassis parameters
    track_width_arg = DeclareLaunchArgument('track_width', default_value='0.5')
    wheel_radius_arg = DeclareLaunchArgument('wheel_radius', default_value='0.1')
    wheel_speed_max_arg = DeclareLaunchArgument('wheel_speed_max', default_value='2.0')
    
    # Goal tolerance
    goal_tolerance_arg = DeclareLaunchArgument('goal_tolerance', default_value='0.1')
    reverse_enabled_arg = DeclareLaunchArgument('reverse_enabled', default_value='false')
    
    # Create node
    chassis_path_follower_node = Node(
        package='gik9dof_controllers',
        executable='chassis_path_follower_node',
        name='chassis_path_follower',
        output='screen',
        parameters=[{
            'controller_mode': LaunchConfiguration('controller_mode'),
            'vx_max': LaunchConfiguration('vx_max'),
            'vx_min': LaunchConfiguration('vx_min'),
            'wz_max': LaunchConfiguration('wz_max'),
            'accel_max': LaunchConfiguration('accel_max'),
            'decel_max': LaunchConfiguration('decel_max'),
            'jerk_limit': LaunchConfiguration('jerk_limit'),
            'lookahead_base': LaunchConfiguration('lookahead_base'),
            'lookahead_gain': LaunchConfiguration('lookahead_gain'),
            'lookahead_min': LaunchConfiguration('lookahead_min'),
            'lookahead_max': LaunchConfiguration('lookahead_max'),
            'kappa_threshold': LaunchConfiguration('kappa_threshold'),
            'vx_reduction': LaunchConfiguration('vx_reduction'),
            'heading_kp': LaunchConfiguration('heading_kp'),
            'feedforward_gain': LaunchConfiguration('feedforward_gain'),
            'track_width': LaunchConfiguration('track_width'),
            'wheel_radius': LaunchConfiguration('wheel_radius'),
            'wheel_speed_max': LaunchConfiguration('wheel_speed_max'),
            'goal_tolerance': LaunchConfiguration('goal_tolerance'),
            'reverse_enabled': LaunchConfiguration('reverse_enabled'),
        }]
    )
    
    return LaunchDescription([
        # Arguments
        controller_mode_arg,
        vx_max_arg,
        vx_min_arg,
        wz_max_arg,
        accel_max_arg,
        decel_max_arg,
        jerk_limit_arg,
        lookahead_base_arg,
        lookahead_gain_arg,
        lookahead_min_arg,
        lookahead_max_arg,
        kappa_threshold_arg,
        vx_reduction_arg,
        heading_kp_arg,
        feedforward_gain_arg,
        track_width_arg,
        wheel_radius_arg,
        wheel_speed_max_arg,
        goal_tolerance_arg,
        reverse_enabled_arg,
        
        # Nodes
        chassis_path_follower_node,
    ])
