import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	params_file = os.path.join(get_package_share_directory(
				'mypkg_acquisition'), 'config/default.yaml')

	acquisition_node = Node(
			package='mypkg_acquisition',
			executable='acquisition_node',
			output='screen',
			parameters=[params_file]
			)

	return LaunchDescription([ acquisition_node ])

