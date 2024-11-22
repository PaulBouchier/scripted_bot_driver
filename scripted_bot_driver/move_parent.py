#!/usr/bin/env python

'''
This is the parent class of several move child classes.
It provides support commont to all the move classes
'''
import time
import numpy as np

import rclpy
import math
from math import pi
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.duration import Duration
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from scripted_bot_interfaces.action import Move
from rcl_interfaces.msg import ParameterDescriptor


loop_rate = 10       # loop rate

# default speed values, which are parameter defaults that can be overriden by parameter cmds
speed_default = 0.35    # driving speed, fwd or back
low_speed_default = 0.15
vel_slew_rate_default = 0.5 / loop_rate  # m/s^2 per loop
rot_speed_default = math.pi/10    # rotating speed, rad/s - 10 sec per revolution
low_rot_speed_default = rot_speed_default/3
rot_slew_rate_default = (rot_speed_default * 3) / loop_rate  # rad/s^2 per loop - slew in 1/3 sec

class MoveParent(Node):

    def __init__(self, node_name):
        super().__init__(node_name)
        self.node_name = node_name

        # initialize parameters
        self.declare_parameter('speed_default_param', speed_default,
                               ParameterDescriptor(description='Default linear full speed, m/s'))
        self.declare_parameter('low_speed_default_param', low_speed_default,
                               ParameterDescriptor(description='Default linear low speed, m/s'))
        self.declare_parameter('vel_slew_rate_default_param', vel_slew_rate_default,
                               ParameterDescriptor(description='Default linear slew rate, m/s^2 per tick'))
        self.declare_parameter('rot_speed_default_param', rot_speed_default,
                               ParameterDescriptor(description='Default rotational full speed, rad/s'))
        self.declare_parameter('low_rot_speed_default_param', low_rot_speed_default,
                               ParameterDescriptor(description='Default rotational low speed, rad/s'))
        self.declare_parameter('rot_slew_rate_default_param', rot_slew_rate_default,
                               ParameterDescriptor(description='Default rotational slew rate, rad/s^2 per tick'))

        # initialize class members
        self.set_defaults()
        self.commandedLinear = 0.0
        self.commandedAngular = 0.0

        # subscribers to robot data
        self.odom = Odometry()
        self.odom_msg_count = 0

        self.map = PoseStamped()
        self.map_msg_count = 0
        
        # Publisher to control robot motion
        self.move_cmd = Twist()
        self.cmd_vel_pub = self.create_publisher(
            Twist, 'cmd_vel', 10
        )

        # set up action
        self.feedback_msg = Move.Feedback()
        self.destroy_subs = False
        
    def send_move_cmd(self, linear, angular):
        self.move_cmd.linear.x = linear
        self.move_cmd.angular.z = angular
        self.commandedLinear = linear
        self.commandedAngular = angular
        self.cmd_vel_pub.publish(self.move_cmd)

    def slew_vel(self, to):
        return self.slew(self.commandedLinear, to, self.vel_slew_rate)

    def slew_rot(self, to):
        return self.slew(self.commandedAngular, to, self.rot_slew_rate)

    def slew(self, current, to, slew_rate):
        diff = to - current
        if diff > slew_rate:
            return current + slew_rate
        if diff < -slew_rate:
            return current - slew_rate
        return to

    def odom_callback(self, odom_msg):
        self.odom = odom_msg
        self.odom_msg_count += 1
        if self.destroy_subs:
            self.destroy_subscription(self.odom_sub)

    
    def is_odom_started(self):
        return self.odom_msg_count > 0

    def map_callback(self, map_msg):
        self.map = map_msg
        self.map_msg_count += 1
        if self.destroy_subs:
            self.destroy_subscription(self.map_sub)
    
    def is_map_started(self):
        return self.map_msg_count > 0

    def set_defaults(self):
        # linear speed parameters
        self.speed = self.get_parameter('speed_default_param').get_parameter_value().double_value
        self.full_speed = self.speed
        self.low_speed = self.get_parameter('low_speed_default_param').get_parameter_value().double_value
        self.vel_slew_rate = self.get_parameter('vel_slew_rate_default_param').get_parameter_value().double_value
        # self.get_logger().info('Linear speed parameter: {:0.2f} m/s'.format(self.speed))

        # rotational speed parameters
        self.rot_speed = self.get_parameter('rot_speed_default_param').get_parameter_value().double_value
        self.full_rot_speed = self.rot_speed
        self.low_rot_speed = self.get_parameter('low_rot_speed_default_param').get_parameter_value().double_value
        self.rot_slew_rate = self.get_parameter('rot_slew_rate_default_param').get_parameter_value().double_value

    # send feedback message to client
    def send_feedback(self, msg, progress):
        self.feedback_msg.feedback_text = msg
        self.feedback_msg.progress = progress
        self.get_logger().info('Feedback: {} {:0.2f}'.format(
            self.feedback_msg.feedback_text,
            self.feedback_msg.progress))
        self._goal_handle.publish_feedback(self.feedback_msg)


    # handle the goal by parsing the move_spec
    def goal_callback(self, goal_request):
        # parse the move_spec specified in the goal
        if (self.parse_argv(goal_request.move_spec) < 0):
            # parsing args failed - return abort
            self.get_logger().error('action failed parsing move_spec')
            return GoalResponse.REJECT

        # start the subscribers, which should only run when an action is requested,
        # on account of the horrendous load they place on a cpu, then delay to let
        # messages start flowing
        self.odom_msg_count = 0
        self.map_msg_count = 0
        self.destroy_subs = False
        odom_cb_group = MutuallyExclusiveCallbackGroup()
        map_cb_group = MutuallyExclusiveCallbackGroup()
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback,
                                                 10, callback_group=odom_cb_group)
        self.map_sub = self.create_subscription(PoseStamped, 'map', self.map_callback,
                                                10, callback_group=map_cb_group)

        rate = self.create_rate(2)
        while (rclpy.ok() and self.odom_msg_count == 0 and self.map_msg_count == 0):
            self.get_logger().info('goal_callback() waiting for odom & map to start')
            rate.sleep()

        self.destroy_rate(rate)

        return GoalResponse.ACCEPT

    # timer callback to call run() in the subclass until done
    def action_run_cb(self):
        if (self.action_complete):
            return  # action_exec_cb hasn't stopped the run timer yet
        if (self.run()):
            # subclass returned true to indicate action is complete
            self.action_complete = True  # indicate to action_exec_cb that action is complete
            self.get_logger().info("run() method reported action complete")
            return
        
        # action not complete, give feedback if needed
        self.loop_count += 1
        if ((self.loop_count % self.feedback_period) == 0):
            text_feedback, progress_feedback = self.get_feedback()
            self.send_feedback(text_feedback, progress_feedback)

    # the action-execute callback
    def action_exec_cb(self, goal_handle):
        self.get_logger().info('{} action_exec_cb called'.format(self.action_name))

        self._goal_handle = goal_handle
        move_results = []
        self.action_complete = False
        self.loop_count = 0
        self.feedback_period = 10    # give feedback every this-many loops
        timer_cb_group = MutuallyExclusiveCallbackGroup()

        # set commanded linear/angular from current linear/angular to pick up current vel from which to slew
        self.commandedLinear = self.odom.twist.twist.linear.x
        self.commandedAngular = self.odom.twist.twist.angular.z

        # start the run_loop_timer calling run() in the subclass
        self.run_loop_timer = self.create_timer(1.0/loop_rate, self.action_run_cb, callback_group=timer_cb_group)

        # let the action_run callback loop perform the move until it indicates complete
        rate = self.create_rate(10)
        while (rclpy.ok() and not self.action_complete):
            # self.get_logger().info('action_exec_cb() waiting for action to complete')
            rate.sleep()

        # destroy the resources that consume cpu after the action is finished
        self.destroy_rate(rate)
        self.destroy_timer(self.run_loop_timer)

        goal_handle.succeed()

        move_result = Move.Result()
        move_results = self.finish_cb()  # get the move results from the subclass
        self.destroy_subs = True
        self.get_logger().info('action finished with success, results: {}'.format(move_results))
        move_result.move_results = move_results
        return move_result

    # start action server
    def create_action_server(self, action_name):
        self.action_name = action_name
        self.get_logger().info('starting action server for action {}'.format(action_name))
        self._goal_handle = None
        self._action_server = ActionServer(
            self,
            Move,
            action_name,
            execute_callback=self.action_exec_cb,
            goal_callback=self.goal_callback,
            #handle_accepted_callback=self.handle_accepted_callback,
            #cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup()
        )

    def quaternion_from_euler(self, ai, aj, ak):
        ai /= 2.0
        aj /= 2.0
        ak /= 2.0
        ci = math.cos(ai)
        si = math.sin(ai)
        cj = math.cos(aj)
        sj = math.sin(aj)
        ck = math.cos(ak)
        sk = math.sin(ak)
        cc = ci*ck
        cs = ci*sk
        sc = si*ck
        ss = si*sk

        q = np.empty((4, ))
        q[0] = cj*sc - sj*cs
        q[1] = cj*ss + sj*cc
        q[2] = cj*cs - sj*sc
        q[3] = cj*cc + sj*ss

        return q
    
    def euler_from_quaternion(self, q):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        x = q.x
        y = q.y
        z = q.z
        w = q.w

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return [roll_x, pitch_y, yaw_z] # in radians

    def normalize(self, angle):     # normalize angle to +/- pi
        if angle > pi:
            angle -= 2 * pi
        if angle < -pi:
            angle += 2 * pi
        return angle

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
    move_parent.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
