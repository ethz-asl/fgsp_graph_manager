import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('graph_manager'),
        'config',
        'manager_config.yaml'
    )

    manager_node = Node(
        package="graph_manager",
        executable="graph_manager_node",
        output={'full': 'screen'},
        emulate_tty=True,
        parameters=[config]
    )
    ld.add_action(manager_node)

    return ld
