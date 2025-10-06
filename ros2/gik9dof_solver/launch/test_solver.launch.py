#!/usr/bin/env python3
"""
test_solver.launch.py
Launch file for testing gik9dof_solver_node with configurable parameters
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    control_rate_arg = DeclareLaunchArgument(
        'control_rate',
        default_value='10.0',
        description='Control loop rate in Hz'
    )
    
    max_solve_time_arg = DeclareLaunchArgument(
        'max_solve_time',
        default_value='0.05',
        description='Maximum solver time in seconds'
    )
    
    distance_lower_arg = DeclareLaunchArgument(
        'distance_lower_bound',
        default_value='0.1',
        description='Lower bound for distance constraint in meters'
    )
    
    distance_weight_arg = DeclareLaunchArgument(
        'distance_weight',
        default_value='1.0',
        description='Weight for distance constraint'
    )
    
    publish_diagnostics_arg = DeclareLaunchArgument(
        'publish_diagnostics',
        default_value='true',
        description='Enable solver diagnostics publishing'
    )
    
    # Create node
    solver_node = Node(
        package='gik9dof_solver',
        executable='gik9dof_solver_node',
        name='gik9dof_solver',
        output='screen',
        parameters=[{
            'control_rate': LaunchConfiguration('control_rate'),
            'max_solve_time': LaunchConfiguration('max_solve_time'),
            'distance_lower_bound': LaunchConfiguration('distance_lower_bound'),
            'distance_weight': LaunchConfiguration('distance_weight'),
            'publish_diagnostics': LaunchConfiguration('publish_diagnostics'),
        }],
        remappings=[
            # Add any topic remappings here if needed
        ]
    )
    
    return LaunchDescription([
        control_rate_arg,
        max_solve_time_arg,
        distance_lower_arg,
        distance_weight_arg,
        publish_diagnostics_arg,
        solver_node
    ])
