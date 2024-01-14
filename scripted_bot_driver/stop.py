#!/usr/bin/env python

import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

#from scripted_bot_driver.move_parent import MoveParent
from scripted_bot_driver.move_parent import MoveParent

loop_rate = 10       # loop rate

class Stop(MoveParent):
    def __init__(self):
        super().__init__('stop')
        self.pause = 0            # pause after stop, in seconds
        self.end_pause_time = self.get_clock().now() # time when pause ends

    def parse_argv(self, argv):
        # check whether a pause argument was supplied
        if (len(argv) > 0):
            try:
                pause_arg = float(argv[0])
                self.pause = pause_arg
                pause_duration = Duration(seconds=pause_arg)
                self.end_pause_time = self.get_clock().now() + pause_duration  # time when pause ends
                self.get_logger().info('Stop is using supplied pause {}'.format(pause_arg))
                return 1
            except ValueError:
                return 0
        return 0            # return number of args consumed

    def print(self):
        print('Stop and pause for {} seconds'.format(self.pause))

    # return True if motion is done
    def run(self):
        if not self.odom_started:
            self.get_logger().error('ERROR: robot odometry has not started - exiting')
            return True

        if self.once:
            self.get_logger().info('Stopping from speed: {}, rot_speed: {} then pausing {} sec'.format(
                self.odom.twist.twist.linear.x,
                self.odom.twist.twist.angular.z,
                self.pause))
            self.once = False

        # loop sending stop commands until both linear & angular request stopped
        if (abs(self.commandedLinear) < 0.01 and
            abs(self.commandedAngular) < 0.01):
            self.send_move_cmd(0.0, 0.0)   # stop the robot
            if self.get_clock().now() > self.end_pause_time:
                self.get_logger().info('Stop paused for {} seconds'.format(self.pause))
                return True
            else:
                return False

        self.send_move_cmd(self.slew_vel(0), self.slew_rot(0))
        return False

def usage():
    print('Usage: stop.py [pause] - ramp down linear & rotational speed & pause if pause is not zero')
    sys.exit()

def main():
    if len(sys.argv) != 1 and len(sys.argv) != 2:
        usage()
    argv_index = 1

    rclpy.init()
    nh = Stop()
    nh.start_spin_thread()   # There's no spinner automatically in a node
    # pause until odometry is received
    while not nh.is_odom_started():
        time.sleep(0.1)
        nh.get_logger().info('slept on odometry')

    r = nh.create_rate(loop_rate)

    argv_index += nh.parse_argv(sys.argv[argv_index:])
    nh.print()

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