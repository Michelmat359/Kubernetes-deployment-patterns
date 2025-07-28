#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Ruta al paquete y argumentos de lanzamiento
    pkg_share = get_package_share_directory('yolo_bringup')

    model_arg = DeclareLaunchArgument(
        'model',
        default_value=os.path.join(pkg_share, 'models', 'yolov8m.pt'),
        description='Ruta al archivo .pt del modelo YOLO'
    )
    tracker_arg = DeclareLaunchArgument(
        'tracker',
        default_value=os.path.join(pkg_share, 'config', 'bytetrack.yaml'),
        description='Ruta al fichero de configuración del tracker'
    )
    device_arg = DeclareLaunchArgument(
        'device',
        default_value='cuda:0',
        description='Dispositivo de inferencia (cpu o cuda:0,1,...)'
    )

    # Nodo de la cámara USB
    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        output='screen',
        parameters=[{
            'video_device': '/dev/video0',
            'image_width': 640,
            'image_height': 480,
            'framerate': 30
        }]
    )

    # Nodo de YOLO
    yolo_node = Node(
        package='yolo_bringup',
        executable='yolo_node',
        name='yolo_node',
        output='screen',
        parameters=[{
            'model': LaunchConfiguration('model'),
            'tracker': LaunchConfiguration('tracker'),
            'device': LaunchConfiguration('device'),
            'enable': True,
            'threshold': 0.5,
            'iou': 0.7,
            'imgsz_height': 480,
            'imgsz_width': 640,
            'half': False,
            'max_det': 300,
            'augment': False,
            'agnostic_nms': False,
            'retina_masks': False,
            'use_tracking': True,
            'use_3d': False,
            'use_debug': True
        }]
    )

    return LaunchDescription([
        model_arg,
        tracker_arg,
        device_arg,
        usb_cam_node,
        yolo_node,
    ])
