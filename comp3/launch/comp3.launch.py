from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    wall_classifier = Node(
        package='comp3',
        executable='wall_classifier',
        name='wall_classifier',
        output='screen',
    )

    leader = Node(
        package='comp3',
        executable='leader',
        name='leader',
        output='screen',
        )
    
    marker_detction = Node(
        package='comp3',
        executable='marker_detection',
        name='marker_detection',
        output='screen',
        )
    

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
    

    return LaunchDescription([
        *nodes,
        wall_classifier,
        leader,
        marker_detction,
        Node(
            package='scoring',
            executable='scoring',
            name='scoring'),
    ])