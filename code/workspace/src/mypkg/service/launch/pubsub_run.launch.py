from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            namespace="my_ws",package="mypkg",executable="pub_exe",output="screen"),
        launch_ros.actions.Node(
            namespace="my_ws",package="mypkg",executable="sub_exe",output="screen"),
    ])
