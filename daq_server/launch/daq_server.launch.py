import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    system_config = os.path.join(get_package_share_directory('daq_server'),
                                 'config/two_of_each.yaml')

    nodes = []

    perfusion_components = Node(package='daq_server',
                                executable='daq_server',
                                output='screen',
                                prefix=["nice -n -20 "],
                                parameters=[system_config]
                                )
    nodes.append(perfusion_components)

    return LaunchDescription(nodes)
