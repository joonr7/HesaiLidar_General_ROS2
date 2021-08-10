import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hesai_lidar',
            node_namespace='hesai_lidar', 
            node_executable='hesai_lidar_node', 
            node_name='hesai_lidar_node', 
            parameters=[
                {'pcap_file': '""'},
                {'server_ip': '192.168.1.201'},
                {'lidar_recv_port': 2368},
                {'gps_port': 10110},
                {'start_angle': 0},
                {'lidar_type': 'PandarQT'},
                {'frame_id': "PandarQT"},
                {'pcldata_type': 0},
                {'publish_type': 'points'},
                {'timestamp_type': '""'},
                {'data_type': '""'},
                {'lidar_correction_file': '../config/correction.csv'}
                ],
            output='screen',
            )
        ])
