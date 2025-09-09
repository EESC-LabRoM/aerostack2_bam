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
Betaflight-Gazebo TF Bridge Node.

This node synchronizes position data between Betaflight SITL and Gazebo
for visual representation. It subscribes to Betaflight position and 
publishes corresponding Gazebo model poses.
"""

__authors__ = 'AeroStack2 Team'
__copyright__ = 'Copyright (c) 2024 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
import tf2_ros


class BetaflightGazeboTFBridge(Node):
    """
    Bridge between Betaflight SITL position and Gazebo model visualization.
    
    This node:
    - Subscribes to Betaflight platform odometry
    - Updates Gazebo model pose to match Betaflight position
    - Provides visual synchronization between flight controller and simulator
    """
    
    def __init__(self):
        super().__init__('betaflight_gazebo_tf_bridge')
        
        # Parameters
        self.declare_parameter('gazebo_model_name', 'betaflight_drone')
        self.declare_parameter('update_rate', 50.0)
        self.declare_parameter('use_sim_time', True)
        
        self.gazebo_model_name = self.get_parameter('gazebo_model_name').value
        self.update_rate = self.get_parameter('update_rate').value
        
        # Gazebo pose service client
        self.pose_service = self.create_client(
            SetEntityPose, 
            '/world/empty/set_pose'
        )
        
        # Subscribe to Betaflight platform odometry
        self.odom_subscription = self.create_subscription(
            Odometry,
            'sensor_measurements/odom',
            self.odom_callback,
            10
        )
        
        # Timer for publishing rate limiting
        self.timer = self.create_timer(1.0 / self.update_rate, self.timer_callback)
        
        # State variables
        self.latest_pose = None
        self.pose_updated = False
        
        self.get_logger().info(f'Betaflight-Gazebo TF Bridge initialized')
        self.get_logger().info(f'Gazebo model: {self.gazebo_model_name}')
        self.get_logger().info(f'Update rate: {self.update_rate} Hz')
        
    def odom_callback(self, msg: Odometry):
        """
        Handle incoming odometry from Betaflight platform.
        
        Args:
            msg: Odometry message from Betaflight platform
        """
        self.latest_pose = msg.pose.pose
        self.pose_updated = True
        
    def timer_callback(self):
        """Update Gazebo model pose at specified rate."""
        if not self.pose_updated or self.latest_pose is None:
            return
            
        if not self.pose_service.service_is_ready():
            self.get_logger().warn('Gazebo pose service not ready', throttle_duration_sec=5.0)
            return
            
        # Create pose request
        request = SetEntityPose.Request()
        request.entity = Entity()
        request.entity.name = self.gazebo_model_name
        request.entity.type = Entity.MODEL
        
        # Copy pose from Betaflight
        request.pose = self.latest_pose
        
        # Send async request
        future = self.pose_service.call_async(request)
        self.pose_updated = False
        
        # Optional: Add callback to handle response
        # future.add_done_callback(self.pose_response_callback)
        
    def pose_response_callback(self, future):
        """
        Handle pose service response.
        
        Args:
            future: Service call future
        """
        try:
            response = future.result()
            if not response.success:
                self.get_logger().warn(f'Failed to update Gazebo pose: {response.status}')
        except Exception as e:
            self.get_logger().error(f'Pose service call failed: {e}')


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    node = BetaflightGazeboTFBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
