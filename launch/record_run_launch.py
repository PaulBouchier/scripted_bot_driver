from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, OpaqueFunction
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
import signal
import time
from datetime import datetime

def generate_launch_description():
    # Declare the launch argument
    arguments = DeclareLaunchArgument(
        'arguments', 
        default_value='',
        description='Arguments for the scripted_mover node'
    )

    # Use LaunchConfiguration to capture the arguments
    arguments_value = LaunchConfiguration('arguments')

    # Get the current date and time for the bag directory name
    current_time = datetime.now().strftime('%Y_%m_%d-%H_%M_%S')
    bag_directory_name = f'rosbag2_{current_time}'

    # Get the output directory from an environment variable, with a default fallback to the user's home directory with a dynamic name
    default_bag_directory = os.path.join('/home', os.getenv('USER'), 'bag_files', bag_directory_name)
    output_directory = os.getenv('ROS2_BAG_OUTPUT_DIRECTORY', default_bag_directory)

    # Start rosbag recording - to support mcap on Humble:
    # sudo apt install ros-humble-rosbag2-storage-mcap 
    start_rosbag_record = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-a', '-s', 'mcap', '-o', output_directory],
        output='screen'
    )

    # Function to stop rosbag recording
    def stop_rosbag_record(context):
        pid = start_rosbag_record.process_details['pid']
        time.sleep(5)  # Delay for 5 seconds before stopping the recording
        os.kill(pid, signal.SIGINT)
        return []

    # Function to launch the scripted mover with parsed arguments
    def launch_scripted_mover(context):
        arguments_str = context.launch_configurations['arguments']
        arguments_list = arguments_str.split()
        # Create the node with arguments, extra arguments will be filtered out at execution time
        scripted_mover_node = Node(
            package='scripted_bot_driver',
            executable='scripted_mover',
            name='scripted_mover',
            output='screen',
            arguments=arguments_list
        )

        return [scripted_mover_node]

    # Function to filter out ROS 2 specific arguments and launch the node
    def execute_filtered_node(context):
        arguments_str = context.launch_configurations['arguments']
        arguments_list = arguments_str.split()
        # Filter out ROS 2 specific arguments
        filtered_arguments_list = [arg for arg in arguments_list if not arg.startswith('--ros-args')]

        # Directly execute the node with filtered arguments
        scripted_mover_node = ExecuteProcess(
            cmd=['ros2', 'run', 'scripted_bot_driver', 'scripted_mover'] + filtered_arguments_list,
            output='screen'
        )

        # Register event handler to stop rosbag recording after the node finishes
        stop_rosbag_record_handler = RegisterEventHandler(
            OnProcessExit(
                target_action=scripted_mover_node,
                on_exit=[OpaqueFunction(function=stop_rosbag_record)]
            )
        )

        return [scripted_mover_node, stop_rosbag_record_handler]

    # OpaqueFunction to launch the filtered execution
    scripted_mover_launcher = OpaqueFunction(function=execute_filtered_node)

    return LaunchDescription([
        arguments,
        start_rosbag_record,
        scripted_mover_launcher,
    ])
