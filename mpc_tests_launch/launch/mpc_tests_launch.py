from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            # namespace='',
            node_executable='static_transform_publisher',
            node_name='static_transform_publisher', 
            arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'world', 'base_link', '__log_level:=debug']
        ),
        Node(
            package='controller',
            # namespace='',
            node_executable='controller',
            node_name='controller', 
            arguments=['__log_level:=debug']
        ),
        Node(
            package='vehicle_sim',
            # namespace='',
            node_executable='vehicle_sim',
            node_name='vehicle_sim', 
            arguments=['__log_level:=debug']
        )
    ])