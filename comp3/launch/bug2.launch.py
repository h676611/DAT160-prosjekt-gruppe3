from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():


    namespaces = ['tb3_0', 'tb3_1']

    nodes = []
    for ns in namespaces:
        nodes.append(Node(
            package='comp3',
            executable='bug2_controller',
            name='bug2_controller',
            namespace=ns,
            output='screen',
        ))
        nodes.append(Node(
            package='comp3',
            executable='wallfollower_controller',
            name='wallfollower_controller',
            namespace=ns,
            output='screen',
        ))
        nodes.append(Node(
            package='comp3',
            executable='gotopoint_controller',
            name='gotopoint_controller',
            namespace=ns,
            output='screen',
        ))
        nodes.append(Node(
            package='comp3',
            executable='robot_controller',
            name='robot_controller',
            namespace=ns,
            output='screen',
        ))

    return LaunchDescription(nodes)