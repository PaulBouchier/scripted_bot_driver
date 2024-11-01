#!/usr/bin/env python

import sys
import time
from math import pow, sqrt

import rclpy
from rclpy.executors import MultiThreadedExecutor

from scripted_bot_driver.move_parent import MoveParent
from scripted_bot_interfaces.msg import DriveStraightDebug
from robot_interfaces.msg import PlatformData

'''
The Seek2Cone class drives toward where it detects the cone, for a limited distance. It completes
the action when it either detects hitting the cone (bumper pressed), or max distance reached
'''

class Seek2Cone(MoveParent):
    def __init__(self):
        super().__init__('seek2cone')

        # subscriber to robot PlatformData
        self.subscription = self.create_subscription(
            PlatformData, 'platform_data', self.platform_data_cb, 10)
        
        # Publisher for debug data
        self.debug_msg = DriveStraightDebug()
        self.debug_pub = self.create_publisher(DriveStraightDebug, 'seek2ConeDebug', 10)

        # create the action server for this move-type
        self.create_action_server('seek2cone')

    def platform_data_cb(self, msg):
        if (not self.bumper_pressed and msg.bumper_pressed):
            self.bumper_pressed = True  # latch bumper_pressed
            self.get_logger().info('Detected bumper pressed!!!')

    def parse_argv(self, argv):
        self.get_logger().info('parsing move_spec {}'.format(argv))
        self.run_once = True
        self.bumper_pressed = False
        self.delta_odom = 0.0
        self.set_defaults()

        if (len(argv) != 1 and len(argv) != 2):
            self.get_logger().fatal('Incorrrect number of args given to Seek2Cone: {}'.format(len(argv)))
            return -1

        try:
            self.distance = float(argv[0])  # max distance to seek
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
        if not self.is_odom_started():
            self.get_logger().error('ERROR: robot odometry has not started - exiting')
            return True

        if self.run_once:
            self.initial_x = self.odom.pose.pose.position.x
            self.initial_y = self.odom.pose.pose.position.y
            self.get_logger().info('seek cone for max {} m at speed: {}'.format(self.distance, self.speed))
            self.get_logger().info('Initial X-Y: {} {}'.format(self.initial_x, self.initial_y))
            self.debug_msg.initial_x = self.initial_x
            self.debug_msg.initial_y = self.initial_y
            self.run_once = False

        delta_x = self.odom.pose.pose.position.x - self.initial_x
        delta_y = self.odom.pose.pose.position.y - self.initial_y
        self.delta_odom = sqrt(pow(delta_x, 2) + pow(delta_y, 2))
        # self.get_logger().info('delta_x: {} delta_y: {} delta_odom: {}'.format(delta_x, delta_y, self.delta_odom))

        if (self.bumper_pressed or self.delta_odom > self.distance):
            self.send_move_cmd(0.0, 0.0)  # hit cone or traveled max distance, slam on the brakes
            self.get_logger().info('traveled: {} m'.format(self.delta_odom))
            return True

        # accelerate to full speed
        self.send_move_cmd(self.slew_vel(self.speed), self.slew_rot(0.0))
        
        # publish debug data
        self.debug_msg.delta_odom = self.delta_odom
        self.debug_msg.speed = self.speed
        self.debug_msg.commanded_linear = self.commandedLinear
        self.debug_msg.commanded_angular = self.commandedAngular
        self.debug_pub.publish(self.debug_msg)

        return False

    def get_feedback(self):
        progress_feedback = self.delta_odom
        text_feedback = 'Seeking cone at {}, traveled {}m'.format(
            self.commandedLinear, progress_feedback)
        return text_feedback, progress_feedback

    # finished action, clean up after a move and reset defaults for next move
    def finish_cb(self):
        self.set_defaults()
        results = [self.delta_odom]
        return results

def main():
    rclpy.init()
    nh = Seek2Cone()

    # Use a MultiThreadedExecutor to enable processing goals concurrently
    executor = MultiThreadedExecutor()
    rclpy.spin(nh, executor=executor)

    nh.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
