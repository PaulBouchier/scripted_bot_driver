#!/usr/bin/env python

import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

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
                self.end_pause_time = self.get_clock.now() + Duration(self.pause)  # time when pause ends
                self.get_logger().info('Stop is using supplied pause {}'.format(self.pause))
                return 1
            except ValueError:
                return 0
        return 0            # return number of args consumed

    def print(self):
        print('Stop and pause for {} seconds'.format(self.pause))

    def run(self):
        if self.once:
            self.get_logger().info('Stopping from speed: {}, rot_speed: {}'.format(
                self.odom.twist.twist.linear.x,
                self.odom.twist.twist.angular.z))
            self.once = False

        # loop sending stop commands until both linear & angular request stopped
        if (abs(self.commandedLinear) < 0.01 and
            abs(self.commandedAngular) < 0.01):
            self.send_move_cmd(0, 0)   # stop the robot
            if self.get_clock().now() > self.end_pause_time:
                self.get_logger.info('Stop paused for {} seconds'.format(self.pause))
                return True
            else:
                return False

        self.send_move_cmd(self.slew_vel(0), self.slew_rot(0))
        return False

    def on_shutdown(self):
        # Always stop the robot when shutting down the node.
        self.get_logger().info("Stopping the robot...")
        self.send_move_cmd(0.0, 0.0)
        time.sleep(1)

def usage():
    print('Usage: stop.py [pause] - ramp down linear & rotational speed & pause if pause is not zero')
    sys.exit()

def main():
    if len(sys.argv) != 1:
        usage()
    argv_index = 1

    rclpy.init()
    stop_node = Stop()

    # Run spin in a thread, make thread daemon so we don't have to join it to exit
    # Provide stop_node so it calls rclpy.spin(stop_node)
    thread = threading.Thread(target=rclpy.spin, args=(stop_node, ), daemon=True)
    thread.start()

    r = stop_node.create_rate(loop_rate)

    try:
        argv_index += stop_node.parse_argv(sys.argv[argv_index:])
        stop_node.print()
        while (rclpy.ok()):
            if stop_node.run():
                break
            r.sleep()

    except Exception as e:
        print(e)
        stop_node.get_logger().info("{} node terminated.".format(__file__))

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    stop_node.on_shutdown()     # stop_node should make things safe
    stop_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()