#!/usr/bin/env python

import sys
import time # Keep time if needed for delays between moves

import rclpy
from rclpy.node import Node

from scripted_bot_interfaces.action import Move
from .single_move_client import SingleMoveClient


class SingleMoveDescriptor():
    def __init__(self, move_type, move_spec):
        self.move_type = move_type
        self.move_spec = move_spec
    def print(self):
        print(self.move_type, self.move_spec)

class ScriptedMover(Node):
    """
    Node to parse command-line arguments into a sequence of moves
    and execute them sequentially using SingleMoveClient.
    """
    def __init__(self):
        super().__init__('scripted_mover') # Node name can be simpler now
        # Instantiate the reusable move client, passing this node
        self.move_client = SingleMoveClient(self)
        self.single_moves = [] # Initialize move list

    def parse_moves(self, argv):
        """ Parses command line arguments into a list of SingleMoveDescriptor objects. """
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
                move_type = 'rotate_odom'
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
    # Removed run_single_move, send_goal, feedback_cb, goal_response_callback, get_result_callback
    # The logic is now handled by SingleMoveClient and the main execution loop.

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
    move_node = None
    exit_code = 0
    try:
        move_node = ScriptedMover()
        move_node.parse_moves(sys.argv[1:])

        # Execute moves sequentially
        for move in move_node.single_moves:
            move_node.get_logger().info(f"--- Starting Move: {move.move_type} {move.move_spec} ---")
            success = move_node.move_client.execute_move(move.move_type, move.move_spec)
            if not success:
                move_node.get_logger().error(f"Move {move.move_type} failed. Aborting sequence.")
                exit_code = 1 # Indicate failure
                break
            else:
                 move_node.get_logger().info(f"--- Completed Move: {move.move_type} ---")
            # Optional: Add a small delay between moves if needed
            # time.sleep(0.5)

        if exit_code == 0:
            move_node.get_logger().info("Successfully completed all moves.")
        else:
            move_node.get_logger().error("Move sequence aborted due to failure.")

    except KeyboardInterrupt:
        move_node.get_logger().info('KeyboardInterrupt, shutting down.')
        exit_code = 1 # Indicate interruption
    except Exception as e:
        if move_node:
            move_node.get_logger().fatal(f"An unexpected error occurred: {e}")
        else:
            print(f"An unexpected error occurred during initialization: {e}")
        exit_code = 1 # Indicate error
    finally:
        # Cleanup
        if move_node:
            move_node.destroy_node()
        rclpy.try_shutdown() # Use try_shutdown to avoid errors if already shut down

    sys.exit(exit_code)


if __name__ == '__main__':
    main()
