from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='async_slam_toolbox_node',
            output='screen',
            parameters=[{
                'use_odom': True,
                'odom_topic': '/odometry/filtered',
                'publish_tf_map': True,
                'odom_frame': 'odom',
                'map_frame': 'map',
                'base_frame': 'base_link',
                'scan_topic': '/scan',
                'use_sim_time': False,
                'minimum_laser_range': 0.15,
                'maximum_laser_range': 12.0,
                'resolution': 0.05
            }]
        )
    ])