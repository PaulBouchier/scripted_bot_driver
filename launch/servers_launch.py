from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='scripted_bot_driver',
            executable='stop',
            name='stop',
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='scripted_bot_driver',
            executable='drive_straight_odom',
            name='drive_straight_odom',
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='scripted_bot_driver',
            executable='drive_straight_map',
            name='drive_straight_map',
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='scripted_bot_driver',
            executable='rotate_odom',
            name='rotate_odom',
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='scripted_bot_driver',
            executable='drive_waypoints',
            name='drive_waypoints',
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='scripted_bot_driver',
            executable='seek2cone',
            name='seek2cone',
            output='screen',
            emulate_tty=True
        )
    ])