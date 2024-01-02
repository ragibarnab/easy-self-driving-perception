from os import path

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    launch_path = path.realpath(__file__)
    launch_dir = path.join(path.dirname(launch_path), '..')
    param_dir = path.join(launch_dir, "param")

    lidar_object_detection_3d_node = Node(
        package = 'object_detector_3d',
        executable = 'lidar_object_detector_3d_node',
        name = 'lidar_object_detector_3d_node',
        parameters=[
            (path.join(param_dir, "lidar_objdet3d_params.yaml")),
        ],
    )

    object_visualizer_node = Node(
        package='visualizer',
        executable='object_visualizer_node',
        name='object_visualizer_node',
    )

    return LaunchDescription([
        lidar_object_detection_3d_node,
        object_visualizer_node
    ])