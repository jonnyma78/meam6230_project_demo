from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    package_share = Path(get_package_share_directory('rcm_modulation_demo'))
    params_file = package_share / 'config' / 'params.yaml'

    return LaunchDescription([
        Node(
            package='rcm_modulation_demo',
            executable='rcm_tracking_node',
            name='rcm_tracking_node',
            output='screen',
            parameters=[str(params_file)],
        )
    ])
