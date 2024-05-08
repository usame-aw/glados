import os

from ament_index_python.packages import get_package_share_directory

import launch
import launch_ros.actions


def generate_launch_description():
    
    publish_stamped_twist = launch.substitutions.LaunchConfiguration('publish_stamped_twist')
    
    config_filepath = launch.substitutions.LaunchConfiguration('config_filepath')

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument('joy_vel', default_value='cmd_vel'),
        
        launch.actions.DeclareLaunchArgument('publish_stamped_twist', default_value='false'),

        launch.actions.DeclareLaunchArgument('config_filepath', default_value=[
            launch.substitutions.TextSubstitution(text=os.path.join(
                get_package_share_directory('glados_teleop'), 'launch', '')),
                    'xbox', launch.substitutions.TextSubstitution(text='.yaml')]),

        launch_ros.actions.Node(
            package='joy', executable='joy_node', name='joy_node',
            parameters=[{
                'dev': '/dev/input/js0',
                'deadzone': 0.2,
                'autorepeat_rate': 20.0,
            }]),
    
        launch_ros.actions.Node(
            package='teleop_twist_joy', executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=[config_filepath, {'publish_stamped_twist': publish_stamped_twist}],
            remappings={('/cmd_vel', launch.substitutions.LaunchConfiguration('joy_vel'))},
            ),
    
    ])