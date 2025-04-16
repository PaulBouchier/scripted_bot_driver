import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
import threading
import time
from action_msgs.msg import GoalStatus

from scripted_bot_interfaces.action import Move

class SingleMoveClient():
    """
    A client class to execute single scripted bot moves using ROS 2 actions.

    This class encapsulates the logic for sending a goal to a specific move action server
    and waiting for its completion.
    """
    def __init__(self, node: Node):
        """
        Initializes the SingleMoveClient.

        Args:
            node: The ROS 2 Node instance to use for creating action clients and logging.
        """
        self.node = node
        self.logger = node.get_logger()

        # Action clients
        self.clients = {
            'stop': ActionClient(node, Move, 'stop'),
            'drive_straight_odom': ActionClient(node, Move, 'drive_straight_odom'),
            'drive_straight_map': ActionClient(node, Move, 'drive_straight_map'),
            'rotate_odom': ActionClient(node, Move, 'rotate_odom'),
            'drive_waypoints': ActionClient(node, Move, 'drive_waypoints'),
            'seek2cone': ActionClient(node, Move, 'seek2cone'),
            'seek2can': ActionClient(node, Move, 'seek2can')
        }

        # Synchronization event for blocking execution
        self.action_complete_event = threading.Event()
        self.last_result = None
        self.last_status = GoalStatus.STATUS_UNKNOWN # Use GoalStatus constants

        # Store futures to prevent garbage collection issues
        self._send_goal_future = None
        self._get_result_future = None

    def _get_client(self, move_type):
        """Gets the appropriate action client for the given move type."""
        # Handle potential mismatch from original script ('rotate' vs 'rotate_odom')
        if move_type == 'rotate':
            move_type = 'rotate_odom'
        return self.clients.get(move_type)

    def execute_move(self, move_type, move_spec):
        """
        Executes a single move and blocks until completion.

        Args:
            move_type: The type of move (e.g., 'drive_straight_odom', 'rotate_odom').
            move_spec: A list of strings specifying the move parameters.

        Returns:
            True if the action succeeded, False otherwise.
        """
        self.logger.info(f"Executing move: {move_type} {move_spec}")
        self.action_complete_event.clear()
        self.last_result = None
        self.last_status = GoalStatus.STATUS_UNKNOWN

        client = self._get_client(move_type)
        if not client:
            self.logger.error(f"Unknown move type: {move_type}")
            return False

        if not client.server_is_ready():
            self.logger.info(f"Waiting for '{move_type}' action server...")
            # Short spin to allow discovery if server just started
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if not client.server_is_ready():
                 # Wait longer if still not found
                if not client.wait_for_server(timeout_sec=5.0):
                    self.logger.error(f"Action server '{move_type}' not available. Aborting move.")
                    return False
            self.logger.info(f"Action server '{move_type}' found.")


        goal_msg = Move.Goal()
        goal_msg.move_spec = move_spec

        self.logger.info(f"Sending goal for {move_type}...")
        self._send_goal_future = client.send_goal_async(
            goal_msg, feedback_callback=self._feedback_cb)

        # Attach the goal response callback
        self._send_goal_future.add_done_callback(self._goal_response_callback)

        # Wait for the action to complete (event set by callbacks)
        # Spin in a loop to process callbacks until the event is set
        while not self.action_complete_event.wait(timeout=0.1):
            # Need rclpy.spin_once to process callbacks
            rclpy.spin_once(self.node, timeout_sec=0.0) # Use minimal timeout for responsiveness

        self.logger.info(f"Move '{move_type}' finished with status: {self._status_to_string(self.last_status)}")
        return self.last_status == GoalStatus.STATUS_SUCCEEDED

    def _feedback_cb(self, feedback_msg):
        """Logs feedback received from the action server."""
        feedback = feedback_msg.feedback
        self.logger.info('Feedback: {} {:0.2f}'.format(feedback.feedback_text, feedback.progress))

    def _goal_response_callback(self, future):
        """Handles the response received after sending a goal."""
        goal_handle = future.result()
        if not goal_handle:
            self.logger.error("Send goal call failed (no handle received).")
            self.last_status = GoalStatus.STATUS_UNKNOWN # Or a custom status?
            self.action_complete_event.set() # Signal completion (failure)
            return
        if not goal_handle.accepted:
            self.logger.error('Goal rejected by server.')
            self.last_status = GoalStatus.STATUS_ABORTED # Treat rejection as an abort
            self.action_complete_event.set() # Signal completion (failure)
            return

        self.logger.info('Goal accepted by server.')
        self._get_result_future = goal_handle.get_result_async()
        # Add the result callback *here*
        self._get_result_future.add_done_callback(self._get_result_callback)

    def _get_result_callback(self, future):
        """Handles the result received when the action completes."""
        result_response = future.result()
        status = result_response.status
        result = result_response.result

        self.last_result = result
        self.last_status = status # Store the numerical status

        status_string = self._status_to_string(status)

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.logger.info(f'Goal succeeded! Result: {result.move_results if result else "None"}')
        elif status == GoalStatus.STATUS_ABORTED:
            self.logger.error(f'Goal aborted by server. Result: {result.move_results if result else "None"}')
        elif status == GoalStatus.STATUS_CANCELED:
            self.logger.warn(f'Goal canceled. Result: {result.move_results if result else "None"}')
        else:
            self.logger.error(f'Goal failed with status: {status_string} ({status}). Result: {result.move_results if result else "None"}')

        self.action_complete_event.set() # Signal completion

    def _status_to_string(self, status):
        """Converts a GoalStatus code to a human-readable string."""
        status_map = {
            GoalStatus.STATUS_UNKNOWN: 'UNKNOWN',
            GoalStatus.STATUS_ACCEPTED: 'ACCEPTED',
            GoalStatus.STATUS_EXECUTING: 'EXECUTING',
            GoalStatus.STATUS_CANCELING: 'CANCELING',
            GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
            GoalStatus.STATUS_CANCELED: 'CANCELED',
            GoalStatus.STATUS_ABORTED: 'ABORTED',
        }
        return status_map.get(status, f'INVALID_STATUS_{status}')
