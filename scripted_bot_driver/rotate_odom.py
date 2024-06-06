#!/usr/bin/env python

import sys
import time
from math import radians, degrees, copysign, sqrt, pow, pi, fmod

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
        self.angle_error = 0.0

    def parse_argv(self, argv):
        if (len(argv) != 1 and len(argv) != 2):
            self.get_logger().fatal('Incorrect number of args given to RotateOdom: {}'.format(len(argv)))
            return -1

        try:
            angle_input = argv[0]
            if angle_input[0] == '+':
                clockwise = False
            elif angle_input[0] == '-':
                clockwise = True
            else:
                clockwise = None
            angle_deg = float(angle_input)  # pick off first arg from supplied list - angle to turn in deg
        except ValueError:
            self.get_logger().fatal('Rotation angle {} must be a float'.format(argv[0]))
            return -1

        if clockwise is not None:
            self.angle_goal = radians(abs(angle_deg)) * angle_correction_sim
            if clockwise:
                self.angle_goal = -self.angle_goal
        else:
            angle = radians(angle_deg)
            if abs(angle) > pi:
                angle = fmod(angle, 2 * pi)
                if angle > pi:
                    angle -= 2 * pi
                elif angle < -pi:
                    angle += 2 * pi
            self.angle_goal = angle * angle_correction_sim

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
        self.get_logger().info('rotate {} deg'.format(self.angle_goal * 180 / pi))

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
        self.euler_heading = euler_angles[2]  # should be 0 - 2pi increasing CCW from East at 0
        self.heading = self.fixup_heading_heading(euler_angles[2], self.angle_to_turn)  # normalize heading to [0, 2pi)

        if self.run_once:
            self.heading_start = self.heading
            self.heading_goal = self.normalize_heading(self.heading_start + self.angle_goal)
            self.rot_stopping = False
            self.full_turns = 0

            self.get_logger().info('start heading: {:.2f}, goal: {:.2f}'.format(self.heading_start, self.heading_goal))
            self.run_once = False
            self.rot_speed = self.full_rot_speed
            self.angular_cmd = 0.0

        # Track the number of full turns
        if self.angle_goal >= 0:
            if self.heading < self.heading_start:
                self.full_turns += 1
        else:
            if self.heading > self.heading_start:
                self.full_turns -= 1

        # Calculate the current angle rotated
        current_angle_rotated = (self.full_turns * 2 * pi) + (self.heading - self.heading_start)

        # Calculate the angle error
        self.angle_error = self.angle_goal - current_angle_rotated

        if self.rot_stopping:
            if abs(self.odom.twist.twist.angular.z) < 0.01:  # wait till we've stopped
                self.run_once = True
                self.get_logger().info('rotated: {} deg to heading {:.2f}'.format((self.heading - self.heading_start) * (180 / pi), self.heading))
                return True
            else:
                self.publish_debug()
                return False

        # slow the rotation speed if we're getting close
        if abs(self.angle_error) < target_close_angle:
            self.rot_speed = self.low_rot_speed

        # stop if we've gone past the goal
        if ((self.angle_goal >= 0 and self.angle_error <= 0) or
            (self.angle_goal < 0 and self.angle_error >= 0)):
            self.rot_speed = 0  # exceeded goal, stop immediately
            self.send_move_cmd(0.0, self.slew_rot(0.0))
            self.rot_stopping = True  # wait until we've stopped before exiting

            self.publish_debug()
            return False

        # Adjust rotation direction based on the angle error
        self.angular_cmd = self.slew_rot(copysign(self.rot_speed, self.angle_error))

        self.send_move_cmd(0.0, self.angular_cmd)

        # publish debug data
        self.publish_debug()

        return False

    def publish_debug(self):
        self.debug_msg.angle_goal = self.angle_goal
        self.debug_msg.heading = self.heading
        self.debug_msg.heading_start = self.heading_start
        self.debug_msg.heading_goal = self.heading_goal
        self.debug_msg.angle_error = self.angle_error
        self.debug_msg.rot_stopping = self.rot_stopping
        self.debug_msg.rot_speed = self.rot_speed
        self.debug_msg.angular_cmd = self.angular_cmd
        self.debug_msg.commanded_linear = self.commandedLinear
        self.debug_msg.commanded_angular = self.commandedAngular
        self.debug_pub.publish(self.debug_msg)

    def normalize_heading(self, angle):  # normalize heading to [0, 2pi)
        angle = fmod(angle, 2 * pi)
        if angle < 0:
            angle += 2 * pi
        return angle

    def start_action_server(self):
        self.create_action_server('rotate_odom')

    def get_feedback(self):
        text_feedback = 'Degrees remaining:'
        progress_feedback = degrees(self.angle_error)  # Set progress_feedback to angle_error
        return text_feedback, progress_feedback

    def finish_cb(self):
        results = [self.angle_goal]
        return results

def main():
    rclpy.init()
    nh = RotateOdom()
    nh.start_action_server()
    nh.start_spin_thread()

    rclpy.spin(nh)

if __name__ == '__main__':
    main()
