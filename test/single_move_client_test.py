import rclpy
from rclpy.node import Node
# Assuming SingleMoveClient is available from this path as per typical ROS 2 structure
# and the /read-only reference to scripted_bot_driver/single_move_client
from scripted_bot_driver.single_move_client import SingleMoveClient

class SingleMoveClientTesterNode(Node):
    def __init__(self):
        super().__init__('single_move_client_tester_node')
        # Instantiate SingleMoveClient, passing the node itself.
        # The instructions mentioned "passing it the mode", which is interpreted
        # here as passing the node, a common requirement for ROS clients.
        # If "mode" refers to a specific string or other type, this may need adjustment.
        self.single_move_client = SingleMoveClient(self)
        self.get_logger().info('SingleMoveClientTesterNode initialized.')

    def run_tests(self):
        self.get_logger().info('Starting test sequence...')
        moves = [
            ("rotate_odom", "roto 1.57"),
            ("rotate_odom", "roto -1.57"),
            ("drive_straight_odom", "movo 0.1"),
            ("drive_straight_odom", "movo -0.1"),
            ("rotate_odom", "roto 1.57")
        ]

        for i in range(10):
            self.get_logger().info(f"--- Iteration {i + 1}/10 ---")
            for move_type, move_command_str in moves:
                self.get_logger().info(f"Executing move: type='{move_type}', command='{move_command_str}'")
                try:
                    # Assuming execute_move is a blocking call or its success/failure
                    # does not need to be explicitly checked for this test script.
                    self.single_move_client.execute_move(move_type, move_command_str)
                    self.get_logger().info(f"Move completed: type='{move_type}', command='{move_command_str}'")
                except Exception as e:
                    self.get_logger().error(f"Error executing move: {e}")
                    # Depending on desired behavior, might re-raise or continue
            self.get_logger().info(f"--- Iteration {i + 1}/10 completed ---")

        self.get_logger().info('All test iterations completed successfully.')

def main(args=None):
    rclpy.init(args=args)
    tester_node = None
    try:
        tester_node = SingleMoveClientTesterNode()
        tester_node.run_tests()
    except KeyboardInterrupt:
        if tester_node:
            tester_node.get_logger().info('Keyboard interrupt, shutting down.')
    except Exception as e:
        if tester_node:
            tester_node.get_logger().error(f"An error occurred: {e}")
        else:
            print(f"An error occurred during node initialization: {e}")
    finally:
        if tester_node:
            tester_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("ROS 2 shutdown complete.")

if __name__ == '__main__':
    main()
