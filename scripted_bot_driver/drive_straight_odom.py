#!/usr/bin/env python

import sys
import time
from math import pow, sqrt

import rclpy
from rclpy.duration import Duration

from scripted_bot_driver.move_parent import MoveParent
from scripted_bot_interfaces.msg import DriveStraightDebug


class DriveStraightOdom(MoveParent):
    def __init__(self):
        super().__init__('drive_straight')
        self.run_once = True

        # Publisher for debug data
        self.debug_msg = DriveStraightDebug()
        self.debug_pub = self.create_publisher(
            DriveStraightDebug, 'drive_straight_debug', 10)

    def parse_argv(self, argv):
        self.get_logger().info('parsing move_spec {}'.format(argv))
        self.run_once = True
        self.delta_odom = 0.0

        if (len(argv) != 1 and len(argv) != 2):
            self.get_logger().error('Incorrrect number of args given to DriveStraightOdom: {}'.format(len(argv)))
            return -1

        try:
            self.distance = float(argv[0])  # pick off first arg from supplied list
            if self.distance < 0:
                self.distance = -self.distance  # distance is always +ve
                self.speed = -self.speed        # go backward
                self.debug_msg.distance = self.distance

            # a supplied speed argument overrides everything
            if len(argv) == 2:
                self.speed = float(argv[1])
                print('Using supplied speed {}'.format(self.speed))
        except ValueError:
            self.get_logger().error('Invalid argument given: {}'.format(argv))
            return -1
        return len(argv)            # return number of args consumed

    def print(self):
        self.get_logger().info('Drive straight with odometry for {} m at speed: {}'.format(self.distance, self.speed))

    # run is called at the rate until it returns true. It does not stop motion on
    # completion - caller is responsible for stopping motion.
    def run(self):
        if not self.is_odom_started():
            self.get_logger().error('ERROR: robot odometry has not started - exiting')
            return True

        if self.run_once:
            self.initial_x = self.odom.pose.pose.position.x
            self.initial_y = self.odom.pose.pose.position.y
            self.get_logger().info('Initial X-Y: {} {}, goal distance: {}'.format(self.initial_x, self.initial_y, self.distance))
            self.debug_msg.initial_x = self.initial_x
            self.debug_msg.initial_y = self.initial_y
            self.run_once = False

        delta_x = self.odom.pose.pose.position.x - self.initial_x
        delta_y = self.odom.pose.pose.position.y - self.initial_y
        self.delta_odom = sqrt(pow(delta_x, 2) + pow(delta_y, 2))
        # self.get_logger().info('x: {} delta_x: {} delta_y: {} delta_odom: {}'.format(self.odom.pose.pose.position.x, delta_x, delta_y, self.delta_odom))

        if (self.delta_odom > self.distance):
            self.get_logger().info('traveled: {} m'.format(self.delta_odom))
            return True

        # accelerate to full speed as long as we haven't reached the goal
        self.send_move_cmd(self.slew_vel(self.speed), self.slew_rot(0.0))
        
        # publish debug data
        self.debug_msg.delta_odom = self.delta_odom
        self.debug_msg.speed = self.speed
        self.debug_msg.commanded_linear = self.commandedLinear
        self.debug_msg.commanded_angular = self.commandedAngular
        self.debug_pub.publish(self.debug_msg)

        return False

    def start_action_server(self):
        self.create_action_server('drive_straight_odom', self.drive_straight_action_exec_cb)

    def drive_straight_action_exec_cb(self):
        self.print()

        loop_period = 0.1
        feedback_period = 10    # give feedback every this-many loops
        loop_count = 0
        self.get_logger().info('starting to call run()')
        try:
            while (rclpy.ok()):
                if self.run():
                    break
                loop_count += 1
                if ((loop_count % feedback_period) == 0):
                    self.send_feedback('Driving straight at {}, traveled: '.format(
                        self.commandedLinear), self.delta_odom)
                time.sleep(loop_period)
        except Exception as e:
            print(e)

        # clean up after a move and reset defaults for next move
        self.get_logger().info('drive_straight_action_exec_cb done()')
        self.run_once = True
        self.set_defaults()

        results = [self.delta_odom]
        return results

    def usage():
        print('Usage: drive_straight.py <distance> [speed] - drive the specified distance forward or backward, with optional speed')
        sys.exit()

def main():
    rclpy.init()
    nh = DriveStraightOdom()
    nh.start_action_server()
    nh.start_spin_thread()

    rclpy.spin(nh)

if __name__ == '__main__':
    main()
