from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='frontier_explorer_3d', executable='nbv_selector',
            name='nbv_selector', output='screen',
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='frontier_explorer_3d', executable='navigator',
            name='navigator', output='screen',
            parameters=[{'use_sim_time': True}],
        ),
    ])
