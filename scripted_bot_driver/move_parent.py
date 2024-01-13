#!/usr/bin/env python

'''
This is the parent class of several move child classes.
It provides support commont to all the move classes
'''
import sys
import time
import threading

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


loop_rate = 10       # loop rate
speed_default = 0.35    # driving speed, fwd or back
low_speed_default = 0.15
vel_slew_rate = 0.25 / loop_rate  # m/s^2 per loop
rot_speed_default = 0.25    # rotating speed, rad/s
low_rot_speed_default = 0.25
rot_slew_rate = 0.5 / loop_rate  # rad/s^2

class MoveParent(Node):

    def __init__(self, node_name):
        super().__init__(node_name)
        self.node_name = node_name
        self.once = True
        self.speed = speed_default
        self.full_speed = speed_default
        self.low_speed = low_speed_default
        self.rot_speed = rot_speed_default
        self.full_rot_speed = rot_speed_default
        self.low_rot_speed = low_rot_speed_default
        self.commandedLinear = 0
        self.commandedAngular = 0

        # subscribers to robot data
        self.odom = Odometry()
        self.odom_started = False
        self.subscription = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)
        
        # Publisher to control robot motion
        self.move_cmd = Twist()
        self.cmd_vel_pub = self.create_publisher(
            Twist, 'cmd_vel', 10
        )

        # Create a thread to run spin_once() so Rate.sleep() works
        self.stop_thread_flag = False
        self.spin_thread = threading.Thread(target=self.spin_thread_entry)


    def send_move_cmd(self, linear, angular):
        self.move_cmd.linear.x = linear
        self.move_cmd.angular.z = angular
        self.commandedLinear = linear
        self.commandedAngular = angular
        self.cmd_vel_pub.publish(self.move_cmd)

    def slew_vel(self, to):
        return self.slew(self.commandedLinear, to, vel_slew_rate)

    def slew_rot(self, to):
        return self.slew(self.commandedAngular, to, rot_slew_rate)

    def slew(self, current, to, slew_rate):
        diff = to - current
        if diff > slew_rate:
            return current + slew_rate
        if diff < -slew_rate:
            return current - slew_rate
        return to

    def odom_callback(self, odom_msg):
        self.odom = odom_msg
        self.odom_started = True
    
    def is_odom_started(self):
        return self.odom_started

    # implement a thread that keeps calling spin_once() so rate.sleep() will work
    def spin_thread_entry(self):
        while(self.stop_thread_flag == False and rclpy.ok()):
            rclpy.spin_once(self, timeout_sec=0.01)
            if (self.stop_thread_flag):
                self.get_logger().debug('Node %s spinner shutting down'%(self.node_name))
                return

    def start_spin_thread(self):
        self.spin_thread.start()

    def shutdown(self):
        self.stop_thread_flag = True
        time.sleep(0.02)    # wait for spin_thread to exit


def main(args=None):
    rclpy.init()

    move_parent = MoveParent('move_parent')

    move_parent.send_move_cmd(1.0, 0.0)
    print('sent move command')
    time.sleep(1)
    move_parent.send_move_cmd(0.0, 0.0)
    print('sent stop command')
    time.sleep(1)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    move_parent.shutdown()
    move_parent.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
