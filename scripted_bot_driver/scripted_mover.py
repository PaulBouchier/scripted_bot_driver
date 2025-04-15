#!/usr/bin/env python

import sys
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
        self.drive_straight_map_client = ActionClient(self, Move, 'drive_straight_map')
        self.rotate_odom_client = ActionClient(self, Move, 'rotate_odom')
        self.drive_waypoints_client = ActionClient(self, Move, 'drive_waypoints')
        self.seek2cone_client = ActionClient(self, Move, 'seek2cone')
        self.seek2can_client = ActionClient(self, Move, 'seek2can')

        self.action_complete = False

    def parse_moves(self, argv):
        current_arg = 0
        self.single_moves = []
        self.current_single_move = 0
        supported_moves = ['stop', 'movo', 'movm', 'roto', 'drive_waypoints', 'seek2cone', 'seek2can']

        print('argv: {}'.format(argv))
        while current_arg != len(argv):
            # extract the move_type from argv
            if argv[current_arg] == 'stop':
                move_type = 'stop'
            elif argv[current_arg] == 'movo':
                move_type = 'drive_straight_odom'
            elif argv[current_arg] == 'movm':
                move_type = 'drive_straight_map'
            elif argv[current_arg] == 'roto':
                move_type = 'rotate'
            elif argv[current_arg] == 'drive_waypoints':
                move_type = 'drive_waypoints'
            elif argv[current_arg] == 'seek2cone':
                move_type = 'seek2cone'
            elif argv[current_arg] == 'seek2can':
                move_type = 'seek2can'
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
        elif(single_move.move_type == 'drive_straight_odom'):
            self.send_goal(self.drive_straight_client, single_move.move_spec)
            self.get_logger().info('sent drive_straight goal')
        elif(single_move.move_type == 'drive_straight_map'):
            self.send_goal(self.drive_straight_map_client, single_move.move_spec)
            self.get_logger().info('sent drive_straight_map goal')
        elif(single_move.move_type == 'rotate'):
            self.send_goal(self.rotate_odom_client, single_move.move_spec)
            self.get_logger().info('sent rotate goal')
        elif(single_move.move_type == 'drive_waypoints'):
                self.send_goal(self.drive_waypoints_client, single_move.move_spec)
                self.get_logger().info('sent drive waypoints goal')
        elif(single_move.move_type == 'seek2cone'):
                self.send_goal(self.seek2cone_client, single_move.move_spec)
                self.get_logger().info('sent seek2cone goal')
        elif(single_move.move_type == 'seek2can'):
                self.send_goal(self.seek2can_client, single_move.move_spec)
                self.get_logger().info('sent seek2can goal')
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
    

def usage():
    print('Usage: scripted_mover.py [commands] - executes the series of move commands provided')
    print('Supported move commands are:')
    print('movo <distance> [speed] - drive straight for <distance> meters using odometry')
    print('movm <distance> [speed] - drive straight for <distance> meters using map nav')
    print('seek2cone <max_distance> [speed] - seek cone for <max_distance> meters')
    print('seek2can <max_distance> - seek can for <max_distance> meters')
    print('roto <target_angle>[d|p] <mode> [angular_speed][d]  [drive_speed] - rotate <angle> radians, or pi*<target_angle> if [p] or degrees if [d], +angle is CCW., mode 1  (default) is shortest_path, mode 2 is strict. Angular speed, if given, allows same modifiers as target_angle.  Drive speed if given is in meters per second, defaults to zero.')
    print('drive_waypoints <target_x> <target_y> [ more_targets ] - drive to a list of targets')
    print('stop [delay]- ramp linear and rotational speed down to 0 with optional pause at end')
    sys.exit()

def main(args=None):
    if len(sys.argv) < 2:
        usage()

    rclpy.init(args=args)

    move_client = ScriptedMover()
    move_client.parse_moves(sys.argv[1:])
    move_client.run_single_move()
    try:
        rclpy.spin(move_client)
    except KeyboardInterrupt:
        print('KeyboardInterrupt - shutting down')


if __name__ == '__main__':
    main()

    
