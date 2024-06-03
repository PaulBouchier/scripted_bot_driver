#!/usr/bin/env python

import sys
import time
from math import radians, copysign, sqrt, pow, pi, asin, atan2

import rclpy
import tf_transformations

from scripted_bot_driver.move_parent import MoveParent
from scripted_bot_interfaces.msg import RotateOdomDebug


target_close_angle = 0.3    # slow down when this close
angle_correction_sim = 0.970

class RotateOdom(MoveParent):
    def __init__(self):
        super().__init__('rotate')
        self.run_once = True

        # Publisher for debug data
        self.debug_msg = RotateOdomDebug()
        self.debug_pub = self.create_publisher(RotateOdomDebug, 'rotate_odom_debug', 10)

    def parse_argv(self, argv):
        if (len(argv) != 1 and len(argv) != 2):
            self.get_logger().fatal('Incorrrect number of args given to RotateOdom: {}'.format(len(argv)))
            return -1

        try:
            angle_deg = float(argv[0])  # pick off first arg from supplied list - angle to turn in deg - +ve = CCW
        except ValueError:
            self.get_logger().fatal('Rotation angle {} must be a float'.format(argv[0]))

        if angle_deg > 180.0 or angle_deg < -180.0:
            self.get_logger().fatal('Rotation angle ({} deg) must be between -180 and +180'.format(argv[0]))
            return -1

        self.angle = angle_deg * (pi / 180) * angle_correction_sim  # angle to rotate through, rad

        # check whether a rot_speed argument was supplied
        if len(argv) > 1:
            try:
                rot_speed_arg = float(argv[1])
                self.rot_speed = rot_speed_arg
                self.get_logger().info('Using supplied rot_speed {}'.format(self.rot_speed))
                return 2
            except ValueError:
                return 1
        return 1            # return number of args consumed

    def print(self):
        self.get_logger().info('rotate {} deg'.format(self.angle * 180 / pi))

    # run is called at the rate until it returns true
    def run(self):
        if not self.is_odom_started():
            self.get_logger().error('ERROR: robot odometry has not started - exiting')
            return True

        # compute heading from /odom
        q = [
            self.odom.pose.pose.orientation.x,
            self.odom.pose.pose.orientation.y,
            self.odom.pose.pose.orientation.z,
            self.odom.pose.pose.orientation.w,
        ]
        euler_angles = tf_transformations.euler_from_quaternion(q)
        self.heading = self.normalize(euler_angles[2])  # range: -pi to +pi

        if self.run_once:
            self.heading_start = self.heading
            self.heading_goal = self.heading + self.angle
            self.crossing_pi = int(self.heading_goal / pi)                 # can take values -1, 0, 1. 0 means not crossing pi
            self.heading_goal -= self.crossing_pi * 2 * pi                       # constrain heading_goal to +/- pi
            self.rot_stopping = False

            self.get_logger().info('start heading: {:.2f}, goal: {:.2f}, crossing2pi: {}'.format(self.heading_start, self.heading_goal, self.crossing_pi))
            self.run_once = False
            self.rot_speed = self.full_rot_speed
            self.angular_cmd = 0.0

        if self.rot_stopping:
            if abs(self.odom.twist.twist.angular.z) < 0.01:     # wait till we've stopped
                self.run_once = True
                self.get_logger().info('rotated: {} deg to heading {:.2f}'.format((self.heading - self.heading_start) * (180 / pi), self.heading))
                return True
            else:
                self.publish_debug()
                return False

        # slow the rotation speed if we're getting close
        if self.crossing_pi == 0:
            if ((self.angle >= 0 and self.heading > (self.heading_goal - target_close_angle)) or
                (self.angle < 0 and self.heading < (self.heading_goal + target_close_angle))):
                self.rot_speed = self.low_rot_speed

        # stop if we've gone past the goal
        if self.crossing_pi == 0 and \
            ((self.angle >= 0 and self.heading >= self.heading_goal) or \
            (self.angle < 0 and self.heading < self.heading_goal)):

            self.rot_speed = 0     # exceeded goal, stop immediately
            self.send_move_cmd(0.0, self.slew_rot(0.0))
            self.rot_stopping = True        # wait until we've stopped before exiting

            self.publish_debug()
            return False

        # +ve angle means turn left. Adjust pi-crossing detection to avoid early exit without movement
        if self.angle >= 0:
            self.angular_cmd = self.slew_rot(self.rot_speed)
            if ((self.crossing_pi != 0) and (self.heading < (self.heading_start - 0.1))):
                self.crossing_pi = 0                 # robot crossed 2pi, now let the end-of-rotate logic work
        else:
            self.angular_cmd = self.slew_rot(-self.rot_speed)
            if ((self.crossing_pi != 0) and (self.heading > (self.heading_start + 0.1))):
                self.crossing_pi = 0                 # robot crossed 2pi, now let the end-of-rotate logic work

        # self.get_logger().info(self.heading)
        self.send_move_cmd(0.0, self.angular_cmd)

        # publish debug data
        self.publish_debug()

        return False

    def publish_debug(self):
        self.debug_msg.angle = self.angle
        self.debug_msg.heading = self.heading
        self.debug_msg.heading_start = self.heading_start
        self.debug_msg.heading_goal = self.heading_goal
        self.debug_msg.crossing_pi = self.crossing_pi
        self.debug_msg.rot_stopping = self.rot_stopping
        self.debug_msg.rot_speed = self.rot_speed
        self.debug_msg.angular_cmd = self.angular_cmd
        self.debug_msg.commanded_linear = self.commandedLinear
        self.debug_msg.commanded_angular = self.commandedAngular
        self.debug_pub.publish(self.debug_msg)

    def normalize(self, angle):     # normalize angle to +/- pi
        if angle > pi:
            angle -= 2 * pi
        if angle < -pi:
            angle += 2 * pi
        return angle

    def start_action_server(self):
        self.create_action_server('rotate_odom')

    def get_feedback(self):
        text_feedback = 'TODO: add feedback'
        progress_feedback = 0.0
        return text_feedback, progress_feedback

    def finish_cb(self):
        results = [self.angle]
        return results

def main():
    rclpy.init()
    nh = RotateOdom()
    nh.start_action_server()
    nh.start_spin_thread()

    rclpy.spin(nh)

if __name__ == '__main__':
    main()

