import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.actions import Node, SetRemap 


def generate_launch_description():
    fra2mo_dir = FindPackageShare('ros2_fra2mo')
    nav2_bringup_launch_file_dir = PathJoinSubstitution(
        [FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py']
    )
    rviz_config_file = os.path.join(get_package_share_directory('ros2_fra2mo'), 'rviz_conf', 'navigation.rviz')


    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    tf_bridge = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_bridge',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'fra2mo_base_footprint']
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=PathJoinSubstitution([fra2mo_dir, 'maps', 'map.yaml']),
        description='Full path to map yaml file to load',
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([fra2mo_dir, 'config', 'navigation.yaml']),
        description='Full path to the ROS2 parameters file to use for all launched nodes',
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation (Gazebo) clock if true'
    )

    # 2. NAV2 BRINGUP CON REMAPPING (Fondamentale per il movimento)
    nav2_bringup_launch = GroupAction(
        actions=[
            # Questo dice a Nav2: "Non scrivere su /cmd_vel, scrivi su /fra2mo/cmd_vel"
            SetRemap(src='/cmd_vel', dst='/fra2mo/cmd_vel'),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([nav2_bringup_launch_file_dir]),
                launch_arguments={
                    'map': map_yaml_file,
                    'params_file': params_file,
                    'use_sim_time': use_sim_time,
                }.items(),
            )
        ]
    )
    
    # Nodo RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file, '--ros-args', '--log-level', 'FATAL'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription(
        [
            declare_map_yaml_cmd,
            declare_params_file_cmd,
            declare_use_sim_time_cmd,
            nav2_bringup_launch,
            rviz_node,
            tf_bridge,
        ]
    )
