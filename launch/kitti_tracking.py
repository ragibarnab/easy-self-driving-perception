from os import path

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    launch_path = path.realpath(__file__)
    launch_dir = path.join(path.dirname(launch_path), '..')
    param_dir = path.join(launch_dir, "param")
    data_dir = path.join(launch_dir, "data")

    kitti_tracking_publisher_node = Node(
        package = 'kitti_publisher',
        executable = 'kitti_tracking_publisher_node',
        name = 'kitti_tracking_publisher_node',
        parameters=[
            (path.join(param_dir, "kitti_params.yaml")),
        ],
    )

    lidar_object_detection_3d_node = Node(
        package = 'object_detector_3d',
        executable = 'lidar_object_detector_3d_node',
        name = 'lidar_object_detector_3d_node',
        parameters=[
            (path.join(param_dir, "lidar_objdet3d_params.yaml")),
        ],
    )

    multi_object_tracker_3d_node = Node(
        package = 'multi_object_tracker_3d',
        executable = 'multi_object_tracker_3d_node',
        name = 'multi_object_tracker_3d_node',
        parameters=[
            (path.join(param_dir, "lidar_objdet3d_params.yaml")),
        ],
    )

    object_visualizer_node = Node(
        package='visualizer',
        executable='object_visualizer_node',
        name='object_visualizer_node',
    )

    rviz = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=['-d' + path.join(data_dir, 'rae.rviz')],
        respawn=True
    )


    return LaunchDescription([
        kitti_tracking_publisher_node,
        lidar_object_detection_3d_node,
        multi_object_tracker_3d_node,
        object_visualizer_node,
        rviz
    ])