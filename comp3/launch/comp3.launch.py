from launch import LaunchDescription
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


from launch.event_handlers import OnProcessStart
from launch.actions import RegisterEventHandler



def generate_launch_description():

    wall_classifier = Node(
        package='comp3',
        executable='wall_classifier',
        name='wall_classifier',
        output='screen',
    )

    from launch.actions import TimerAction

    leader = TimerAction(
        period=5.0,  # wait 5 seconds for tb3_2 to spawn
        actions=[Node(
            package='comp3',
            executable='leader',
            name='leader',
            output='screen'
        )]
        )
        
    marker_detction = Node(
        package='comp3',
        executable='marker_detection',
        name='marker_detection',
        output='screen',
        )
    

    namespaces = ['tb3_0', 'tb3_1', 'tb3_2']

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


    other_pkg_share = get_package_share_directory('multi_robot_challenge_23')
    other_launch = os.path.join(other_pkg_share, 'launch', 'spawn_robot.launch.py')

    include_other = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(other_launch),
        launch_arguments={
            'namespace': 'tb3_2',
            'x': '-0.5',
            'y': '0.0',
            'use_sim_time': 'false'
        }.items()
    )

    
    

    return LaunchDescription([
        *nodes,
        wall_classifier,
        leader,
        marker_detction,
        Node(
            package='scoring',
            executable='scoring',
            name='scoring'),
        include_other
    ])