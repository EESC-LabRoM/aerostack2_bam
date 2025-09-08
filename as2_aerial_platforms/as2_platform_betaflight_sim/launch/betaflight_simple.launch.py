#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='drone0',
            description='Drone namespace'
        ),
        
        # Launch our Betaflight platform bridge only
        Node(
            package='as2_platform_betaflight_sim',
            executable='betaflight_sim_node',
            name='platform',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'control_modes_file': PathJoinSubstitution([
                    FindPackageShare('as2_platform_betaflight_sim'), 
                    'config', 
                    'control_modes.yaml'
                ]),
            }],
        ),
    ])
