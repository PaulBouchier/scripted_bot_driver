#!/usr/bin/env python

import sys
import threading
import time
from math import pow, sqrt

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from scripted_bot_driver.move_parent import MoveParent

loop_rate = 10       # loop rate

class DriveStraightOdom(MoveParent):
    def __init__(self):
        super().__init__('drive_straight')
        self.initial_x = 0.0
        self.initial_y = 0.0

    def parse_argv(self, argv):
        self.distance = float(argv[0])  # pick off first arg from supplied list
        if self.distance < 0:
            self.distance = -self.distance  # distance is always +ve
            self.speed = -self.speed        # go backward

        # a supplied speed argument overrides everything
        if len(argv) > 1:
            try:
                self.speed = float(argv[1])
                print('Using supplied speed {}'.format(self.speed))
                return 2
            except ValueError:
                return 1
        return 1            # return number of args consumed

    def print(self):
        self.get_logger().info('Drive straight with odometry for {} m'.format(self.distance))

    # run is called at the rate until it returns true. It does not stop motion on
    # completion - caller is responsible for stopping motion.
    def run(self):
        if not self.odom_started:
            self.get_logger().error('ERROR: robot odometry has not started - exiting')
            return True

        if self.once:
            self.initial_x = self.odom.pose.pose.position.x
            self.initial_y = self.odom.pose.pose.position.y
            self.get_logger().info('Initial X-Y: {} {}, goal distance: {}'.format(
                self.initial_x, self.initial_y, self.distance))
            self.once = False

        delta_x = self.odom.pose.pose.position.x - self.initial_x
        delta_y = self.odom.pose.pose.position.y - self.initial_y
        delta_odom = sqrt(pow(delta_x, 2) + pow(delta_y, 2))

        if (delta_odom > self.distance):
            self.get_logger().info('traveled: {} m'.format(delta_odom))
            return True

        # accelerate to full speed as long as we haven't reached the goal
        if self.distance >= 0:
            self.send_move_cmd(self.slew_vel(self.speed), self.slew_rot(0.0))
        else:
            self.send_move_cmd(self.slew_vel(-self.speed), self.slew_rot(0.0))

        return False

def usage():
    print('Usage: drive_straight.py <distance> [speed] - drive the specified distance forward or backward, with optional speed')
    sys.exit()

def main():
    if len(sys.argv) != 2 and len(sys.argv) != 3:
        usage()
    argv_index = 1

    rclpy.init()
    nh = DriveStraightOdom()
    nh.start_spin_thread()   # There's no spinner automatically in a node
    # pause until odometry is received
    while not nh.is_odom_started():
        time.sleep(0.1)
        nh.get_logger().info('slept on odometry')

    r = nh.create_rate(loop_rate)

    argv_index += nh.parse_argv(sys.argv[argv_index:])
    nh.print()
    time.sleep(0.1)

    try:
        while (rclpy.ok()):
            if nh.run():
                break
            r.sleep()

    except Exception as e:
        print(e)
        nh.get_logger().info("{} node terminated.".format(__file__))

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    nh.shutdown()     # nh should make things safe
    nh.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
