import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from scripted_bot_interfaces.action import Move


class ScriptedMover(Node):

    def __init__(self):
        super().__init__('scripted_mover_client')
        self._stop_client = ActionClient(self, Move, 'stop')
        self._drive_straight_client = ActionClient(self, Move, 'drive_straight_odom')

    def send_stop_goal(self, move_spec):
        goal_msg = Move.Goal()
        goal_msg.move_spec = move_spec

        self._stop_client.wait_for_server()

        self._send_goal_future = self._stop_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_cb)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def send_drive_straight_goal(self, move_spec):
        goal_msg = Move.Goal()
        goal_msg.move_spec = move_spec

        self._drive_straight_client.wait_for_server()

        self._send_goal_future = self._drive_straight_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_cb)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_cb(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('{} {}'.format(feedback.feedback_text, feedback.progress))


    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected - Exiting')
            rclpy.shutdown()
            return

        self.get_logger().info('Goal accepted')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.move_results))
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)

    move_client = ScriptedMover()

    future = move_client.send_drive_straight_goal(['2'])

    rclpy.spin(move_client)


if __name__ == '__main__':
    main()