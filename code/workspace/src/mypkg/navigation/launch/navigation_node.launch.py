import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	params_file = os.path.join(get_package_share_directory(
				'mypkg_navigation'), 'config/default.yaml')

	navigation_node = Node(
			package='mypkg_navigation',
			executable='navigation_node',
			output='screen',
			parameters=[params_file]
			)

	return LaunchDescription([ navigation_node ])

