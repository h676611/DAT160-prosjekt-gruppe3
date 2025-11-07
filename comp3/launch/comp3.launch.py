from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    bug2_controller = Node(
        package='comp3',
        executable='bug2_controller',
        name='bug2_controller',
        output='screen',
    )

    wallfollower_controller = Node(
        package='comp3',
        executable='wallfollower_controller',
        name='wallfollower_controller',
        output='screen',
    )

    gotopoint_controller = Node(
        package='comp3',
        executable='gotopoint_controller',
        name='gotopoint_controller',
        output='screen',
    )

    robot_controller = Node(
        package='comp3',
        executable='robot_controller',
        name='robot_controller',
        output='screen',
    )

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
    

    return LaunchDescription([
        bug2_controller,
        wallfollower_controller,
        gotopoint_controller,
        robot_controller,
        wall_classifier,
        leader
    ])