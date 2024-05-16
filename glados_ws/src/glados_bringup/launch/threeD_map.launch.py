from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():

    return LaunchDescription([
        
        Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='mapping',
            output='screen',
            parameters=[
                {"frame_id": "odom"},
                {"base_frame_id": "base_footprint"},
                {"resolution": 0.001},
                {"sensor_model.max_range": -1.0},
                {"sensor_model.hit": 0.8},
                {"sensor_model.miss": 0.2},
                {"filter_speckles": True},
                
                # {"filter_ground_plane": True},
                # {"ground_filter.distance": 0.10},
                # {"ground_filter.angle": 0.15},
                # {"ground_filter.plane_distance": 0.10},
                
                # {"point_cloud_min_z": -99.0},
                # {"point_cloud_max_z": 99.0},
                
                # {"occupancy_min_z": -99.0},
                # {"occupancy_max_z": 21.0},
                
            ],
            
            remappings=[
                ('/cloud_in', "lidar_pc"),
            ]),
          
    ])