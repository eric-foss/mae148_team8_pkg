from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mae148_team8_pkg',
            executable='gps_node',
            output='screen'),])