from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():

    return LaunchDescription([
        
        # Lidar Bringup
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('glados_lidar'),
                    'launch',
                    'lidar_bringup.launch.py'
                ])
            ]),
        ),
        
        # Teleop Bringup
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('glados_teleop'),
                    'launch',
                    'teleop.launch.py'
                ])
            ]),
        ),
        
        Node(
            package='glados_teleop',
            executable='teleoperate',
            name='teleoperate',
            output='screen'),

        # Create TF tree
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('glados_bringup'),
                    'launch',
                    'create_tf.launch.py'
                ])
            ]),
        ),
        

    ])