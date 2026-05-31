from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    cfg = os.path.join(
        get_package_share_directory('frontier_explorer_py'),
        'config', 'frontier_extractor.yaml')

    return LaunchDescription([
        Node(
            package='frontier_explorer_py',
            executable='frontier_extractor',
            name='frontier_extractor',
            output='screen',
            parameters=[cfg, {'use_sim_time': True}],
        ),
    ])
