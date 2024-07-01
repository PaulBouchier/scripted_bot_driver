#!/usr/bin/env python

import string
import sys
import time
from math import radians, degrees, copysign, sqrt, pow, pi, fmod

import rclpy
import tf_transformations

from scripted_bot_driver.move_parent import MoveParent
from scripted_bot_driver.anglr import  Angle
from scripted_bot_driver.AngleHunter import AngleHunter
from scripted_bot_interfaces.msg import RotateOdomDebug


target_close_angle = 0.3    # slow down when this close in radians
angle_correction_sim = 1.0  # can use this to hack the cutoff to mitigate over rotation? was 0.970

class RotateOdom(MoveParent):
    def __init__(self):
        super().__init__('rotate')
        #  set up loop timer
        self.last_time = self.get_clock().now()
        self.estimated_loop_rate = 0.0
        self.alpha = 0.1  # Smoothing factor for exponential moving average
        self.commandedAngular = 0.0
        self.rot_stopping = False
        self.run_once = True

        # Publisher for debug data
        self.debug_msg = RotateOdomDebug()
        self.debug_pub = self.create_publisher(RotateOdomDebug, 'rotate_odom_debug', 10)
        self.angle_error = 0.0

        # Initialize timer to call run method at 30 Hz - no worky
        #self.timer = self.create_timer(1.0 / 30.0, self.run)

    def parse_argv(self, argv):
        
        if len(argv) not in [1, 2, 3]:
            self.get_logger().fatal('Incorrect number of args given to RotateOdom: {}'.format(len(argv)))
            return -1
        self.shortest_path = True
        try:
            sp = argv[1]
            if sp == '-1':
                self.shortest_path = False
                
            self.angle_goal = self.parse_angle_rad(argv[0])  # pick off first arg from supplied list
            self.get_logger().info(f'argv[0] {argv[0]}')
        except ValueError:
            self.get_logger().fatal('Rotation angle {} must be convertible to a float'.format(argv[0]))
            return -1

        # check whether a rot_speed argument was supplied. if ending in 'd' convert from degrees
        if len(argv) > 1:
            try:
                rot_speed_arg = self.parse_angle_rad(argv[2])
                self.get_logger().info(f'argv[2] {argv[2]}')
                self.full_rot_speed = rot_speed_arg
                self.get_logger().info(f'full_rot_speed {self.full_rot_speed}')
                self.get_logger().info('Using supplied rot_speed {} radians'.format(self.rot_speed))
                return 2
            except ValueError:
                return 1
        
        return 1  # return number of args consumed
    
    def parse_angle_rad(self, angle_str):
        """ convert an angle string to a float value in radians. angles ending in 'd' will be converted from degrees"""
        angle_rad = 0 # default return value if an angle isn't provided
        if angle_str:
            if angle_str[-1] in ['d', 'D']: 
                angle_rad = radians(float(angle_str[:-1]))
            else: 
                angle_rad = float (angle_str)

        return angle_rad

    def print(self):
        self.get_logger().info(f'rotate {self.angle_goal} rad, {degrees(self.angle_goal)} deg')

    # run is called regularly until it returns true
    def run(self):
        current_time = self.get_clock().now()
        time_diff = (current_time - self.last_time).nanoseconds / 1e9  # Convert nanoseconds to seconds
        self.last_time = current_time

        if time_diff > 0:
            instant_loop_rate = 1.0 / time_diff
            self.estimated_loop_rate = self.alpha * instant_loop_rate + (1.0 - self.alpha) * self.estimated_loop_rate

        rot_slew_rate = 0.5 / self.estimated_loop_rate  # Adjusted slew rate based on estimated loop rate

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
        self.heading = Angle(euler_angles[2]).normalized().radians  # normalize heading to [0, 2pi), though it should be already
        
        if self.run_once:
            self.heading_start = self.heading
            self.get_logger().info(f'self.heading_start {self.heading_start}')
            #initialize an AngleHunter to track heading error
            self.ah=AngleHunter(self.angle_goal*angle_correction_sim, self.heading_start, self.shortest_path)
            self.angle_goal = self.ah.get_target_radians() 
            self.heading_goal = self.ah.get_target_radians()
            self.rot_stopping = False
            
            self.get_logger().info(f'start goal: {self.heading_goal:.2f}')
            self.run_once = False
            self.rot_speed = self.full_rot_speed
            self.get_logger().info(f'self.rot_speed runonce {self.rot_speed}')
            self.angular_cmd = 0.0

        #  update our turn error
        self.angle_error=self.ah.update(self.heading).radians
        
        if self.rot_stopping:
            if abs(self.odom.twist.twist.angular.z) < 0.01:  # wait till we've stopped
                self.run_once = True
                self.get_logger().info(f'rotated: {self.ah.get_cumul_degrees()} deg to heading {self.heading}')
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
            self.rot_speed = 0.0  # exceeded goal, stop immediately
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
        self.debug_msg.angle_goal = self.angle_goal #todo seems redundant with heading_goal?
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

    def slew_rot(self, to):
        rot_slew_rate = 0.5 / self.estimated_loop_rate  # Dynamic slew rate
        return self.slew(self.commandedAngular, to, rot_slew_rate)

    def slew(self, current, to, slew_rate):
        diff = to - current
        if diff > slew_rate:
            return current + slew_rate
        if diff < -slew_rate:
            return current - slew_rate
        return to

    def start_action_server(self):
        self.create_action_server('rotate_odom')

    def get_feedback(self):
        text_feedback = 'Degrees remaining:' 
        text_feedback = f'{repr(self.ah)}\nloop rate: {self.estimated_loop_rate:.2f} Hz'
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
