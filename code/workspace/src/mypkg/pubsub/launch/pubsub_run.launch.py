from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            namespace="my_ws",package="mypkg_pubsub",executable="pub_exe",output="screen"),
        launch_ros.actions.Node(
            namespace="my_ws",package="mypkg_pubsub",executable="sub_exe",output="screen"),
    ])
