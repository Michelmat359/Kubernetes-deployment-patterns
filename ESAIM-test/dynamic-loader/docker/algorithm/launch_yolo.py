#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('yolo_bringup')

    return LaunchDescription([
        DeclareLaunchArgument(
            'model',
            default_value=os.path.join(pkg_share, 'models', 'yolov8m.pt'),
            description='Ruta al modelo .pt'
        ),
        DeclareLaunchArgument(
            'tracker',
            default_value=os.path.join(pkg_share, 'config', 'bytetrack.yaml'),
            description='Ruta al config del tracker'
        ),
        DeclareLaunchArgument(
            'device',
            default_value='cuda:0',
            description='Dispositivo (cpu o cuda:0,1,...)'
        ),

        Node(
            package='yolo_bringup',
            executable='yolo_node',
            name='yolo_node',
            output='screen',
            parameters=[{
                'model': LaunchConfiguration('model'),
                'tracker': LaunchConfiguration('tracker'),
                'device': LaunchConfiguration('device'),
                # … resto de parámetros iguales al monolítico …
            }]
        )
    ])
