#!/usr/bin/env python

import sys
import time
from math import pow, sqrt, atan2, pi

import rclpy
from rclpy.executors import MultiThreadedExecutor

from scripted_bot_driver.move_parent import MoveParent
from scripted_bot_interfaces.msg import DriveStraightDebug


class DriveStraightMap(MoveParent):
    def __init__(self):
        super().__init__('drive_straight_map')

        # Publisher for debug data
        self.debug_msg = DriveStraightDebug()
        self.debug_pub = self.create_publisher(DriveStraightDebug, 'drive_straight_debug', 10)

        # create the action server for this move-type
        self.create_action_server('drive_straight_map')

    def parse_argv(self, argv):
        self.get_logger().info('parsing move_spec {}'.format(argv))
        self.run_once = True
        self.delta_map = 0.0
        self.set_defaults()

        if (len(argv) != 1 and len(argv) != 2):
            self.get_logger().fatal('Incorrrect number of args given to DriveStraightMap: {}'.format(len(argv)))
            return -1

        try:
            self.distance = float(argv[0])  # pick off first arg from supplied list
            self.debug_msg.distance = self.distance
            if self.distance < 0:
                self.distance = -self.distance  # distance is always +ve
                self.speed = -self.speed        # go backward

            # a supplied speed argument overrides everything
            if len(argv) == 2:
                self.speed = float(argv[1])
                self.get_logger().info('Using supplied speed {}'.format(self.speed))
        except ValueError:
            self.get_logger().error('Invalid argument given: {}'.format(argv))
            return -1
        return len(argv)            # return number of args consumed


    # run is called at the rate until it returns true. It does not stop motion on
    # completion - caller is responsible for stopping motion.
    def run(self):
        if not self.is_map_started():
            self.get_logger().error('ERROR: robot map topic has not started - exiting')
            return True

        if self.run_once:
            self.initial_x = self.map.pose.position.x
            self.initial_y = self.map.pose.position.y
            self.get_logger().info('Drive straight using map for {} m at speed: {}'.format(self.distance, self.speed))
            self.get_logger().info('Initial X-Y: {} {}, goal distance: {}'.format(self.initial_x, self.initial_y, self.distance))
            self.debug_msg.initial_x = self.initial_x
            self.debug_msg.initial_y = self.initial_y
            self.run_once = False

        delta_x = self.map.pose.position.x - self.initial_x
        delta_y = self.map.pose.position.y - self.initial_y
        self.delta_map = sqrt(pow(delta_x, 2) + pow(delta_y, 2))
        # self.get_logger().info('delta_x: {} delta_y: {} delta_map: {}'.format(delta_x, delta_y, self.delta_map))

        if (self.delta_map > self.distance):
            self.send_move_cmd(0.0, 0.0)  # traveled required distance, slam on the brakes
            heading = atan2(delta_y, delta_x)

            # get heading from /odom
            euler_angles = self.euler_from_quaternion(self.odom.pose.pose.orientation)
            compass_heading = self.normalize(euler_angles[2])

            self.get_logger().info('traveled: {:.2f} m with gps heading {:.2f} compass heading {:.2f}'.format(
                self.delta_map, heading, compass_heading))
            return True

        # accelerate to full speed as long as we haven't reached the goal
        self.send_move_cmd(self.slew_vel(self.speed), self.slew_rot(0.0))
        
        # publish debug data
        self.debug_msg.delta_odom = self.delta_map
        self.debug_msg.speed = self.speed
        self.debug_msg.commanded_linear = self.commandedLinear
        self.debug_msg.commanded_angular = self.commandedAngular
        self.debug_pub.publish(self.debug_msg)

        return False

    def get_feedback(self):
        progress_feedback = self.delta_map
        text_feedback = 'Driving straight at {}, traveled {}m'.format(
            self.commandedLinear, progress_feedback)
        return text_feedback, progress_feedback

    # finished action, clean up after a move and reset defaults for next move
    def finish_cb(self):
        self.set_defaults()
        results = [self.delta_map]
        return results

def main():
    rclpy.init()
    nh = DriveStraightMap()

    try:
        # Use a MultiThreadedExecutor to enable processing goals concurrently
        executor = MultiThreadedExecutor()
        rclpy.spin(nh, executor=executor)
    except KeyboardInterrupt:
        print("Shutting down after KeyboardInterrupt")

if __name__ == '__main__':
    main()
