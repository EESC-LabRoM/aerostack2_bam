#!/usr/bin/env python3

# Copyright 2024 Universidad Politécnica de Madrid
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
Hybrid Betaflight SITL + Gazebo visualization launch file.

This launch file provides:
- Betaflight SITL for flight dynamics and MSP protocol
- Gazebo for 3D visualization and sensor simulation
- MSP platform bridge for AeroStack2 integration
- Optional TF bridge for position synchronization
"""

__authors__ = 'AeroStack2 Team'
__copyright__ = 'Copyright (c) 2024 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

import os

from ament_index_python.packages import get_package_share_directory
from as2_core.declare_launch_arguments_from_config_file import DeclareLaunchArgumentsFromConfigFile
from as2_core.launch_configuration_from_config_file import LaunchConfigurationFromConfigFile
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def get_package_config_file():
    """Return the package config file."""
    package_folder = get_package_share_directory('as2_platform_betaflight_sim')
    return os.path.join(package_folder, 'config/platform_config_file.yaml')


def get_gazebo_config_file():
    """Return the Gazebo simulation config file."""
    package_folder = get_package_share_directory('as2_platform_betaflight_sim')
    return os.path.join(package_folder, 'config/gazebo_simulation.json')


def get_node(context, *args, **kwargs) -> list:
    """
    Get MSP platform node and optional TF bridge.

    :param context: Launch context
    :type context: LaunchContext
    :return: List with platform node and optional TF bridge
    :rtype: list
    """
    # Get namespace
    namespace = LaunchConfiguration('namespace').perform(context)
    
    nodes = []
    
    # MSP Platform Node
    platform_node = Node(
        package='as2_platform_betaflight_sim',
        executable='as2_platform_betaflight_sim_node',
        name='platform',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        emulate_tty=True,
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'control_modes_file': LaunchConfiguration('control_modes_file'),
            },
            LaunchConfigurationFromConfigFile(
                'platform_config_file',
                default_file=get_package_config_file())
        ]
    )
    nodes.append(platform_node)
    
    # TF Bridge Node (optional) - synchronizes Betaflight SITL position with Gazebo
    tf_bridge_node = Node(
        package='as2_platform_betaflight_sim',
        executable='betaflight_gazebo_tf_bridge',
        name='tf_bridge',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'gazebo_model_name': LaunchConfiguration('gazebo_model_name'),
                'update_rate': 50.0  # Hz
            }
        ],
        condition=IfCondition(LaunchConfiguration('enable_tf_bridge'))
    )
    nodes.append(tf_bridge_node)
    
    return nodes


def generate_launch_description() -> LaunchDescription:
    """
    Entry point for launch file.

    :return: Launch description
    :rtype: LaunchDescription
    """
    package_folder = get_package_share_directory('as2_platform_betaflight_sim')
    control_modes = os.path.join(package_folder, 'config/control_modes.yaml')
    
    # Gazebo simulation launch
    gazebo_assets_folder = get_package_share_directory('as2_gazebo_assets')
    gazebo_simulation_launch = os.path.join(gazebo_assets_folder, 'launch/launch_simulation.py')
    
    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument('log_level',
                              description='Logging level',
                              default_value='info'),
        DeclareLaunchArgument('use_sim_time',
                              description='Use simulation clock if true',
                              default_value='true'),
        DeclareLaunchArgument('namespace',
                              description='Drone namespace',
                              default_value=EnvironmentVariable(
                                  'AEROSTACK2_SIMULATION_DRONE_ID', default_value='drone_sim_0')),
        DeclareLaunchArgument('control_modes_file',
                              default_value=control_modes,
                              description='Platform control modes file'),
        DeclareLaunchArgument('gazebo_config_file',
                              default_value=get_gazebo_config_file(),
                              description='Gazebo simulation configuration file'),
        DeclareLaunchArgument('gazebo_model_name',
                              default_value='betaflight_drone',
                              description='Name of the drone model in Gazebo'),
        DeclareLaunchArgument('enable_gazebo',
                              default_value='true',
                              description='Enable Gazebo simulation'),
        DeclareLaunchArgument('enable_tf_bridge',
                              default_value='false',
                              description='Enable TF bridge between Betaflight and Gazebo'),
        DeclareLaunchArgument('headless',
                              default_value='false',
                              description='Run Gazebo in headless mode'),
        DeclareLaunchArgument('betaflight_host',
                              default_value='localhost',
                              description='Betaflight SITL host'),
        DeclareLaunchArgument('betaflight_port',
                              default_value='5761',
                              description='Betaflight SITL MSP port'),
        
        # Platform configuration file arguments
        DeclareLaunchArgumentsFromConfigFile(
            name='platform_config_file', source_file=get_package_config_file(),
            description='Configuration file'),
        
        # Gazebo Simulation (conditional)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_simulation_launch),
            launch_arguments={
                'simulation_config_file': LaunchConfiguration('gazebo_config_file'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'headless': LaunchConfiguration('headless'),
                'verbose': 'false',
                'run_on_start': 'true'
            }.items(),
            condition=IfCondition(LaunchConfiguration('enable_gazebo'))
        ),
        
        # MSP Platform + Optional TF Bridge
        OpaqueFunction(function=get_node)
    ])
