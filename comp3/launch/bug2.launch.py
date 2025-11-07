from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    namespace = LaunchConfiguration('namespace', default='')

    declare_ns_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace for all nodes'
    )

    bug2_controller = Node(
        package='comp3',
        executable='bug2_controller',
        name='bug2_controller',
        namespace=namespace,
        output='screen',
    )

    wallfollower_controller = Node(
        package='comp3',
        executable='wallfollower_controller',
        name='wallfollower_controller',
        namespace=namespace,
        output='screen',
    )

    gotopoint_controller = Node(
        package='comp3',
        executable='gotopoint_controller',
        name='gotopoint_controller',
        namespace=namespace,
        output='screen',
    )

    robot_controller = Node(
        package='comp3',
        executable='robot_controller',
        name='robot_controller',
        namespace=namespace,
        output='screen',
    )

    return LaunchDescription([
        declare_ns_arg,
        bug2_controller,
        wallfollower_controller,
        gotopoint_controller,
        robot_controller,
    ])