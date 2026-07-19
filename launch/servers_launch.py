from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    speed_default_arg = DeclareLaunchArgument(
        'speed_default',
        default_value='0.20',
        description='Default linear full speed, m/s'
    )

    rot_speed_default_arg = DeclareLaunchArgument(
        'rot_speed_default',
        default_value='0.60',
        description='Default rotational full speed, rad/s'
    )

    drive_waypoints_node = Node(
        package='scripted_bot_driver',
        executable='drive_waypoints',
        name='drive_waypoints',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'speed_default': LaunchConfiguration('speed_default'),
            'rot_speed_default': LaunchConfiguration('rot_speed_default')
        }]
    )

    return LaunchDescription([
        speed_default_arg,
        rot_speed_default_arg,
        drive_waypoints_node,
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
        #Node(
        #    package='scripted_bot_driver',
        #    executable='seek2cone',
        #    name='seek2cone',
        #    output='screen',
        #    emulate_tty=True,
        #    parameters= [{
        #            'speed_default_param':          0.15,
        #            'low_speed_default_param':      0.15,
        #            'rot_speed_default_param':      0.25,
        #            'back_and_aim_param':           False,
        #            'use_oakd_param':               False
        #    }]
        #)
    ])
