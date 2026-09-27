import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('qcar_gazebo')
    cartographer_config_dir = os.path.join(pkg, 'config', 'cartographer')
    configuration_basename = 'qcar_2d.lua'

    launch_sim = LaunchConfiguration('launch_sim')

    return LaunchDescription([
        DeclareLaunchArgument(
            'launch_sim',
            default_value='false',
            description='Bring up Gazebo + robot_state_publisher + controllers as well'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, 'launch', 'qcar_gazebo.launch.py')
            ),
            condition=IfCondition(launch_sim)
        ),

        # real-hardware equivalent when launch_sim:=false
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, 'launch', 'qcar_visualize.launch.py')
            ),
            condition=UnlessCondition(launch_sim)
        ),

        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': launch_sim}],
            arguments=[
                '-configuration_directory', cartographer_config_dir,
                '-configuration_basename', configuration_basename
            ]
        ),
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': launch_sim}],
            arguments=['-resolution', '0.05']
        ),
    ])
