#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='frontier_safety',
            executable='collision_avoidance',
            name='collision_avoidance',
            output='screen',
            parameters=[{
                'cloud_topic':        '/simple_drone/velodyne_points',
                'input_cmd':          '/simple_drone/cmd_vel_raw',
                'output_cmd':         '/simple_drone/cmd_vel',
                'cloud_frame':        'velodyne_link',
                'influence_radius':   3.0,
                'emergency_distance': 0.8,
                'self_radius':        0.25,
                'repulsion_gain':     0.8,
                'max_linear_speed':   1.0,
                'control_rate':       20.0,
                'cmd_timeout':        0.5,
                'avoid_vertical':     True,
                'histogram_sectors':  72,
                'gaussian_sigma_deg': 10.0,
                'smoothing_alpha':    0.4,
            }],
        ),
    ])
