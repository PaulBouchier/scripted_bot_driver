#!/usr/bin/env python

import sys
import time

import rclpy
from rclpy.duration import Duration

from scripted_bot_driver.move_parent import MoveParent


class Stop(MoveParent):
    def __init__(self):
        super().__init__('stop')

    def parse_argv(self, argv):
        self.run_once = True
        self.pause = 0.0            # pause after stop, in seconds
        self.end_pause_time = self.get_clock().now() # time when pause ends

        if (len(argv) != 0 and len(argv) != 1):
            self.get_logger().error('Incorrrect number of args given to Stop: {}'.format(len(argv)))
            return -1

        # check whether a pause argument was supplied
        if (len(argv) > 0):
            try:
                pause_arg = float(argv[0])
                self.pause = pause_arg
                pause_duration = Duration(seconds=pause_arg)
                self.end_pause_time = self.get_clock().now() + pause_duration  # time when pause ends
                self.get_logger().info('Stop will use supplied pause {}'.format(pause_arg))
                return 1
            except ValueError:
                self.get_logger().error('Invalid argument given: {}'.format(argv))
                return -1
        return 0            # return number of args consumed

    def print(self):
        print('Stop and pause for {} seconds'.format(self.pause))

    # return True if motion is done
    def run(self):
        if not self.is_odom_started():
            self.get_logger().error('ERROR: robot odometry has not started - exiting')
            return True

        if self.run_once:
            self.get_logger().info('Stopping from speed: {}, rot_speed: {} then pausing {} sec'.format(
                self.odom.twist.twist.linear.x,
                self.odom.twist.twist.angular.z,
                self.pause))
            self.run_once = False
            self.start_time = time.time()

        # loop sending stop commands until both linear & angular request stopped
        linear = self.odom.twist.twist.linear.x
        angular = self.odom.twist.twist.angular.z
        if (abs(linear) < 0.01 and abs(angular) < 0.01):
            self.get_logger().debug('linear: {} angular {}'.format(linear, angular))
            self.send_move_cmd(self.slew_vel(0.0), self.slew_rot(0.0))   # stop the robot
            if self.get_clock().now() > self.end_pause_time:
                self.get_logger().info('Stop paused for {} seconds'.format(self.pause))
                return True
            else:
                return False

        self.send_move_cmd(self.slew_vel(0.0), self.slew_rot(0.0))
        return False

    def start_action_server(self):
        self.create_action_server('stop')

    def get_feedback(self):
        progress = time.time() - self.start_time
        return 'slowing or pausing: ', progress

    def finish_cb(self):
        results = [self.pause]
        return results

    def usage():
        print('Usage: stop.py [pause] - ramp down linear & rotational speed & pause if pause is not zero')
        sys.exit()

def main():
    rclpy.init()

    nh = Stop()
    nh.start_action_server()
    nh.start_spin_thread()

    rclpy.spin(nh)


if __name__ == '__main__':
    main()