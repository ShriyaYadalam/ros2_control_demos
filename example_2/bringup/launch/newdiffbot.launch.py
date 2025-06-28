import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get package directory
    pkg_share = get_package_share_directory('amr_urdf_v3')
    
    # Path to URDF file
    urdf_file = os.path.join(pkg_share, 'urdf', 'amr_urdf_v3.urdf')
    
    # Read URDF file
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

        
    
    # Path to RViz config
    rviz_config_file = os.path.join(pkg_share, 'urdf.rviz')
    
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'model',
            default_value='',
            description='Model argument (for compatibility)'
        ),
        
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),
        
        # Joint State Publisher GUI
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),
        
        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else []
        )
    ])
