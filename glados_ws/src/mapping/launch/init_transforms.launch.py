from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        
        Node(
             package='tf2_ros',
             executable='static_transform_publisher',
             arguments = ['--x', '0.0', '--y', '0.0', '--z', '0.0', 
                          '--yaw', '0.0', '--pitch', '0.0', '--roll', '0.0', 
                          '--frame-id', 'base_link', '--child-frame-id', 'chassis_link']
        ),

        Node(
             package='tf2_ros',
             executable='static_transform_publisher',
             arguments = ['--x', '0.0', '--y', '-0.109', '--z', '0.0105', 
                          '--yaw', '0.0', '--pitch', '0.0', '--roll', '0.0', 
                          '--frame-id', 'chassis_link', '--child-frame-id', 'right_wheel_link']
        ),

        Node(
             package='tf2_ros',
             executable='static_transform_publisher',
             arguments = ['--x', '0.0', '--y', '0.109.', '--z', '0.0105', 
                          '--yaw', '0.0', '--pitch', '0.0', '--roll', '0.0', 
                          '--frame-id', 'chassis_link', '--child-frame-id', 'left_wheel_link']
        ),
        ############################################
     #    Node(
     #         package='tf2_ros',
     #         executable='static_transform_publisher',
     #         arguments = ['--x', '1.0', '--y', '-0.109.', '--z', '0.10', 
     #                      '--yaw', '0.0', '--pitch', '0.0', '--roll', '0.0', 
     #                      '--frame-id', 'chassis_link', '--child-frame-id', 'rotation_axis']
     #    ),

        Node(
             package='tf2_ros',
             executable='static_transform_publisher',
             arguments = ['--x', '0.0', '--y', '0.080598', '--z', '0.0', 
                          '--yaw', '0.0', '--pitch', '0.0', '--roll', '0.0', 
                          '--frame-id', 'rotation_point', '--child-frame-id', 'intersection_point']
        ),

        Node(
             package='tf2_ros',
             executable='static_transform_publisher',
             arguments = ['--x', '0.0', '--y', '0.0.', '--z', '0.03879', 
                          '--yaw', '1.57', '--pitch', '0.0', '--roll', '0.0', 
                          '--frame-id', 'intersection_point', '--child-frame-id', 'lidar_frame']
        ),
    ])