from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='demo_pipeline', executable='dataset_player', name='dataset_player', output='screen'),
        Node(package='demo_pipeline', executable='semantic_segmentation', name='semantic_segmentation', output='screen'),
        Node(package='demo_pipeline', executable='ground_filter', name='ground_filter', output='screen'),
        Node(package='safety_gateway', executable='safety_gateway', name='safety_gateway', output='screen'),
        Node(package='adaptive_grid', executable='adaptive_grid', name='adaptive_grid', output='screen'),
        Node(package='rosbridge_server', executable='rosbridge_websocket', name='rosbridge_websocket', output='screen', parameters=[{'port': 9090}]),
    ])