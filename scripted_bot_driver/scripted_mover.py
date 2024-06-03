#!/usr/bin/env python

import sys
import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from scripted_bot_interfaces.action import Move

class SingleMoveDescriptor():
    def __init__(self, move_type, move_spec):
        self.move_type = move_type
        self.move_spec = move_spec
    def print(self):
        print(self.move_type, self.move_spec)


class ScriptedMover(Node):
    def __init__(self):
        super().__init__('scripted_mover_client')
        self.stop_client = ActionClient(self, Move, 'stop')
        self.drive_straight_client = ActionClient(self, Move, 'drive_straight_odom')
        self.rotate_odom_client = ActionClient(self, Move, 'rotate_odom')
        self.drive_waypoints_client = ActionClient(self, Move, 'drive_waypoints')

        # Create a thread to process supplied arguments and send action requests
        #self.action_thread = threading.Thread(target=self.action_thread_entry)

        self.action_complete = False

    def parse_moves(self, argv):
        current_arg = 0
        self.single_moves = []
        self.current_single_move = 0
        supported_moves = ['stop', 'movo', 'roto', 'drive_waypoints']

        print('argv: {}'.format(argv))
        while current_arg != len(argv):
            # extract the move_type from argv
            if argv[current_arg] == 'stop':
                move_type = 'stop'
            elif argv[current_arg] == 'movo':
                move_type = 'drive_straight'
            elif argv[current_arg] == 'roto':
                move_type = 'rotate'
            elif argv[current_arg] == 'drive_waypoints':
                move_type = 'drive_waypoints'
            else:
                self.get_logger().fatal('Error - unknown move type {}'.format(argv[current_arg]))
                rclpy.shutdown()
                sys.exit()
            self.get_logger().info('found move type {}'.format(move_type))

            # extract move_spec args (if any)
            move_spec = []
            current_arg += 1
            for arg in argv[current_arg:]:
                if arg in supported_moves:
                    # if any arg is a known move_type, finish move_spec accumulation
                    break
                move_spec.append(arg)
                current_arg += 1
            move = SingleMoveDescriptor(move_type, move_spec)
            print('parsed move: {} {}'.format(move.move_type, move.move_spec))
            self.single_moves.append(move)
        self.get_logger().info('Parsed {} moves:'.format(len(self.single_moves)))
        for move in self.single_moves:
            move.print()    

    # Pull the next single_move off the array and start it
    def run_single_move(self):
        single_move = self.single_moves[self.current_single_move]
        self.current_single_move += 1   # prep for next move

        if(single_move.move_type == 'stop'):
            self.send_goal(self.stop_client, single_move.move_spec)
            self.get_logger().info('sent stop goal')
        elif(single_move.move_type == 'drive_straight'):
            self.send_goal(self.drive_straight_client, single_move.move_spec)
            self.get_logger().info('sent drive_straight goal')
        elif(single_move.move_type == 'rotate'):
            self.send_goal(self.rotate_odom_client, single_move.move_spec)
            self.get_logger().info('sent rotate goal')
        elif(single_move.move_type == 'drive_waypoints'):
                self.send_goal(self.drive_waypoints_client, single_move.move_spec)
                self.get_logger().info('sent drive waypoints goal')
        else:
            self.get_logger().fatal('ERROR: requested to run unsupported move {}'.format(single_move.move_type))
            rclpy.shutdown()

    def send_goal(self, client, move_spec):
        goal_msg = Move.Goal()
        goal_msg.move_spec = move_spec

        client.wait_for_server()

        self._send_goal_future = client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_cb)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

        
    def feedback_cb(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('{} {:0.2f}'.format(feedback.feedback_text, feedback.progress))


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
        self.action_complete = True
        if self.current_single_move == len(self.single_moves):
            self.get_logger().info('Finished moves, exiting')
            rclpy.shutdown()
        else:
            self.run_single_move()
    
    def start_action_thread(self):
        self.action_thread.start()

    def action_thread_entry(self):
        self.get_logger().info('Starting thread to process move commands')
        future = self.send_drive_straight_goal(['2'])
        while not self.action_complete:
            time.sleep(1)

        self.get_logger().info('all moves completed - shutting down')
        rclpy.shutdown()


def usage():
    print('Usage: scripted_mover.py [commands] - executes the series of move commands provided')
    print('Supported move commands are:')
    print('movo <distance> [speed] - drive straight for <distance> meters')
    #print('arc <angle> <radius> <f | b>')
    print('roto <angle> [speed] - rotate <angle> degrees, +ve is CCW. Speed, if given, sets direction.')
    print('drive_waypoints <target_x> <target_y> [ more_targets ] - drive to a list of targets')
    print('stop [delay]- ramp linear and rotational speed down to 0 with optional pause at end')
    sys.exit()

def main(args=None):
    if len(sys.argv) < 2:
        usage()

    rclpy.init(args=args)

    move_client = ScriptedMover()
    # move_client.start_action_thread()
    move_client.parse_moves(sys.argv[1:])
    move_client.run_single_move()
    #future = move_client.send_drive_straight_goal(['2'])
    rclpy.spin(move_client)


if __name__ == '__main__':
    main()

    
