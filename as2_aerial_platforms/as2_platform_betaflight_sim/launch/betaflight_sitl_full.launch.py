#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
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
        
        DeclareLaunchArgument(
            'fcu_url',
            default_value='udp://localhost:5761@',  # Corrected port for Betaflight SITL
            description='FCU connection URL for MAVROS'
        ),
        
        # Launch MAVROS to connect to Betaflight SITL
        Node(
            package='mavros',
            executable='mavros_node',
            name='mavros',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[{
                'fcu_url': LaunchConfiguration('fcu_url'),
                'gcs_url': '',
                'target_system_id': 1,
                'target_component_id': 1,
                'fcu_protocol': 'v2.0',
                'use_sim_time': False,  # Set to false for SITL testing
            }],
            remappings=[
                # Standard MAVROS topics
                ('/mavros/imu/data', '/mavros/imu/data'),
                ('/mavros/setpoint_raw/attitude', '/mavros/setpoint_raw/attitude'),
            ]
        ),
        
        # Launch our Betaflight platform bridge
        Node(
            package='as2_platform_betaflight_sim',
            executable='as2_platform_betaflight_sim_node',
            name='platform',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[{
                'use_sim_time': False,  # Set to false for SITL testing
                'control_modes_file': PathJoinSubstitution([
                    FindPackageShare('as2_platform_betaflight_sim'), 
                    'config', 
                    'control_modes.yaml'
                ]),
            }],
            remappings=[
                # Connect to MAVROS topics
                ('/mavros/imu/data', '/mavros/imu/data'),
                ('/mavros/setpoint_raw/attitude', '/mavros/setpoint_raw/attitude'),
            ]
        ),
    ])
