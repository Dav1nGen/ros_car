import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


params_file = os.path.join(get_package_share_directory('qrcode_detector'),'config/qrcode.yaml')
qrcode_detector_node = Node(
        package='qrcode_detector',
        executable='qrcode_detector',
        parameters=[params_file],
        )


def generate_launch_description():
    return LaunchDescription([
        qrcode_detector_node
    ])
