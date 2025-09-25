#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directories
    limo_simulation_dir = get_package_share_directory('limo_simulation')
    limo_control_dir = get_package_share_directory('limo_control')
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    goal_x_arg = DeclareLaunchArgument(
        'goal_x',
        default_value='2.0',
        description='Goal X position'
    )
    
    goal_y_arg = DeclareLaunchArgument(
        'goal_y',
        default_value='2.0',
        description='Goal Y position'
    )
    
    kp_linear_arg = DeclareLaunchArgument(
        'kp_linear',
        default_value='1.0',
        description='Proportional gain for linear velocity'
    )
    
    kp_angular_arg = DeclareLaunchArgument(
        'kp_angular',
        default_value='2.0',
        description='Proportional gain for angular velocity'
    )
    
    # Include the headless simulation launch file
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(limo_simulation_dir, 'launch', 'limo_headless.launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items()
    )
    
    # P Controller node
    p_controller_node = Node(
        package='limo_control',
        executable='p_controller',
        name='p_controller',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'goal_x': LaunchConfiguration('goal_x'),
            'goal_y': LaunchConfiguration('goal_y'),
            'kp_linear': LaunchConfiguration('kp_linear'),
            'kp_angular': LaunchConfiguration('kp_angular'),
            'max_linear_vel': 0.5,
            'max_angular_vel': 1.0,
            'goal_tolerance': 0.1
        }]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        goal_x_arg,
        goal_y_arg,
        kp_linear_arg,
        kp_angular_arg,
        simulation_launch,
        p_controller_node
    ])