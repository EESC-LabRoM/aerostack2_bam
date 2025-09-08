#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='drone0',
            description='Drone namespace'
        ),
        
        Node(
            package='as2_platform_betaflight_sim',
            executable='betaflight_sim_node',
            name='platform',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[{
                'use_sim_time': True,
            }],
            remappings=[
                # Remap to connect with Gazebo
                ('sensor_measurements/imu', 'sensor_measurements/imu'),
                ('mavros/setpoint_raw/attitude', 'mavros/setpoint_raw/attitude'),
            ]
        ),
    ])
