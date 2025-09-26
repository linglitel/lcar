from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lcar_gps',
            executable='gps',
            name='gps_node'
            # 可以在这里添加参数：parameters=[{'param_name': value}]
        )
    ])