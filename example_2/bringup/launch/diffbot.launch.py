# # Copyright 2020 ros2_control Development Team
# #
# # Licensed under the Apache License, Version 2.0 (the "License");
# # you may not use this file except in compliance with the License.
# # You may obtain a copy of the License at
# #
# #     http://www.apache.org/licenses/LICENSE-2.0
# #
# # Unless required by applicable law or agreed to in writing, software
# # distributed under the License is distributed on an "AS IS" BASIS,
# # WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# # See the License for the specific language governing permissions and
# # limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource 
from launch.conditions import IfCondition, UnlessCondition 
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
import xacro
import os
from ament_index_python.packages import get_package_share_directory 

def generate_launch_description():
    pkg_share = FindPackageShare(package='ros2_control_demo_example_2').find('ros2_control_demo_example_2')
    pkg_urdf = FindPackageShare(package= 'amr_urdf_v3').find('amr_urdf_v3') 

    default_model_path = os.path.join(pkg_share, 'description', 'urdf', 'diffbot.urdf')
    #default_model_path = os.path.join(pkg_urdf, 'urdf', 'amr_urdf_v3.urdf')

    robot_controllers_path = os.path.join(pkg_share, 'bringup', 'config', 'diffbot_controllers.yaml')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'config.rviz')     
    # default_map_yaml_path = '/home/shriya/ros2_ws/src/2wheeldrive/maps/mapfinal.yaml'
    
    print(f"Package share path: {pkg_share}")
    print(f"Controllers file path: {robot_controllers_path}")
    print(f"RVIZ path: {default_rviz_config_path}") 
    # print(f"Map file path: {default_map_yaml_path}")
    # print(f"Map file exists: {os.path.exists(default_map_yaml_path)}")

    robot_description_content = Command(['xacro ', LaunchConfiguration('model')])
    robot_description = ParameterValue(robot_description_content, value_type=str)

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': robot_description}, robot_controllers_path],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
            ("/diffbot_base_controller/cmd_vel", "/cmd_vel"), 
        ],
    )
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher', 
        parameters=[{'robot_description': robot_description}]
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diffbot_base_controller", "--controller-manager", "/controller_manager"],
    )    

    # joint_state_publisher_node = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher', 
    #     name='joint_state_publisher',  
    #     parameters=[{
    #         'robot_description': ParameterValue(Command(['xacro ', default_model_path]), value_type=str),
    #         'use_sim_time': True
    #     }],
    #     condition=UnlessCondition(LaunchConfiguration('gui'))
    # )
    # joint_state_publisher_gui_node = Node(
    #     package='joint_state_publisher_gui',
    #     executable='joint_state_publisher_gui',
    #     name='joint_state_publisher_gui',
    #     parameters=[{'use_sim_time': True}],
    #     condition=IfCondition(LaunchConfiguration('gui'))
    # )
    # gaz = ExecuteProcess(
    #     cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', 
    #          '-world', '/home/shriya/world1.world'], 
    #     output='screen'
    # )
    # spawn_entity = Node(
    #     package='gazebo_ros',
    #     executable='spawn_entity.py',
    #     parameters=[{'use_sim_time': True}],
    #     arguments=['-entity', '2wheeldrive', '-topic', 'robot_description', '-x', '-5', '-y', '-5'],
    #     output='screen'
    # ) 

    # cmd_vel_bridge = Node( 
    # package='topic_tools',
    # executable='relay',
    # name='cmd_vel_bridge',
    # arguments=['/cmd_vel', '/demo/cmd_vel'],
    # parameters=[{'use_sim_time': False}], 
    # output='screen'
    # ) 

    # relay_odom = Node(
    # package='topic_tools',
    # executable='relay',
    # name='odom_relay',
    # arguments=['/demo/odom', '/odom'],
    # parameters=[{'use_sim_time': False}],
    # output='screen'  
    # )

    # map_server = Node( 
    #     package='nav2_map_server',
    #     executable='map_server',
    #     name='map_server',
    #     output='screen',
    #     parameters=[{
    #         'use_sim_time': True,   
    #         'yaml_filename': default_map_yaml_path,
    #         'topic_name': 'map',
    #         'frame_id': 'map'
    #     }],
    # )

    # static_tf_map_odom = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='static_tf_pub_map_odom',
    #     arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
    #     parameters=[{'use_sim_time': True}]
    # ) 


    # robot_localization_node = Node(
    #     package='robot_localization',
    #     executable='ekf_node',
    #     name='ekf_filter_node',
    #     output='screen',
    #     parameters=[os.path.join(pkg_share, 'config/ekf.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
  
    # )

    # amcl = Node(
    #     package='nav2_amcl',  
    #     executable='amcl',
    #     name='amcl', 
    #     output='screen',   
    #     parameters=[{
    #         'use_sim_time': True,
    #         'alpha1': 0.2,
    #         'alpha2': 0.2,
    #         'alpha3': 0.2,
    #         'alpha4': 0.2,
    #         'base_frame_id': 'base_link',
    #         'global_frame_id': 'map',
    #         'odom_frame_id': 'odom',
    #         'scan_topic': 'scan',
    #         'transform_tolerance': 0.1,
    #         'max_particles': 2000,
    #         'min_particles': 500,
    #         'set_initial_pose': True, 
    #         'initial_pose.x': 10.0,
    #         'initial_pose.y': 10.0,
    #         'initial_pose.z': 0.0,
    #         'initial_pose.yaw': 0.0,            
    #     }] 
    # )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True, 
            'node_names': ['map_server', 'amcl']   
        }]
    )

    rviz_node = Node(
         package='rviz2',
         executable='rviz2',
         name='rviz2',
         output='screen',
         parameters=[{'use_sim_time': False}], 
         arguments=['-d', LaunchConfiguration('rvizconfig')],
     )  

    return LaunchDescription([
        DeclareLaunchArgument(
            name='model',
            default_value=default_model_path,
            description='Absolute path to robot model file'
        ),
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            name='rvizconfig', 
            default_value=default_rviz_config_path,
            description='Absolute path to rviz config file'
        ), 



        # Start Gazebo and robot first
        # gaz,

        control_node,
        robot_state_publisher_node,

        TimerAction(
            period=2.0,
            actions=[robot_controller_spawner]
        ),

        TimerAction(
            period=3.0,
            actions=[joint_state_broadcaster_spawner]
        ),

        # TimerAction(
        #     period=4.0,
        #     actions=[robot_localization_node]
        # ),

        # TimerAction(
        #     period=5.0,
        #     actions=[map_server, amcl, lifecycle_manager]
        # ),

        TimerAction(
             period=5.0,
             actions=[rviz_node]
        )

        # cmd_vel_bridge, 
        # relay_odom,
        # joint_state_publisher_node,
        # spawn_entity,
    ])
