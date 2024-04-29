from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():


    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('glados_lidar'),
                    'launch',
                    'lidar_driver.launch.py'
                ])
            ]),
        ),

        Node(
            package='glados_lidar',
            executable='laser_filter',
            name='laser_filter',
            output='screen'),

        Node(
            package='glados_lidar',
            executable='2pointcloud',
            name='topointcloud',
            output='screen'),
    
    ])