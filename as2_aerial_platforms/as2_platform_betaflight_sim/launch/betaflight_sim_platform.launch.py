#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directory
    package_dir = get_package_share_directory('as2_platform_betaflight_sim')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='drone0',
            description='Drone namespace'
        ),
        
        DeclareLaunchArgument(
            'msp_connection',
            default_value='udp:localhost:5761',
            description='MSP connection string (udp:host:port or /dev/ttyUSB0)'
        ),
        
        DeclareLaunchArgument(
            'msp_baud_rate',
            default_value='115200',
            description='MSP baud rate for serial connections'
        ),
        
        Node(
            package='as2_platform_betaflight_sim',
            executable='betaflight_sim_node',
            name='platform',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[
                os.path.join(package_dir, 'config', 'control_modes.yaml'),
                {
                    'use_sim_time': False,  # MSP protocol works in real-time
                    'msp_connection': LaunchConfiguration('msp_connection'),
                    'msp_baud_rate': LaunchConfiguration('msp_baud_rate'),
                    'control_modes_file': os.path.join(package_dir, 'config', 'control_modes.yaml'),
                }
            ],
        ),
    ])
