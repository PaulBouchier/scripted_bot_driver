#!/usr/bin/env python

'''
This is the parent class of several move child classes.
It provides support commont to all the move classes
'''
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from scripted_bot_interfaces.action import Move


loop_rate = 10       # loop rate
speed_default = 0.35    # driving speed, fwd or back
low_speed_default = 0.15
vel_slew_rate = 0.5 / loop_rate  # m/s^2 per loop
rot_speed_default = 0.25    # rotating speed, rad/s
low_rot_speed_default = 0.25
rot_slew_rate = 0.5 / loop_rate  # rad/s^2

class MoveParent(Node):

    def __init__(self, node_name):
        super().__init__(node_name)
        self.node_name = node_name
        self.set_defaults()
        self.commandedLinear = 0.0
        self.commandedAngular = 0.0

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

        # set up action
        self.feedback_msg = Move.Feedback()

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

    def set_defaults(self):
        self.speed = speed_default
        self.full_speed = speed_default
        self.low_speed = low_speed_default
        self.rot_speed = rot_speed_default
        self.full_rot_speed = rot_speed_default
        self.low_rot_speed = low_rot_speed_default

    # implement a thread that keeps calling spin_once() so rate.sleep() will work
    def spin_thread_entry(self):
        while(self.stop_thread_flag == False and rclpy.ok()):
            rclpy.spin_once(self, timeout_sec=0.01)
            if (self.stop_thread_flag):
                self.get_logger().debug('Node %s spinner shutting down'%(self.node_name))
                return

    def start_spin_thread(self):
        self.spin_thread.start()

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
        if (not self.is_odom_started()):
            # odom not started - return abort
            self.get_logger().error('stop action failed - odometry not running')
            return GoalResponse.REJECT
        # parse the move_spec specified in the goal
        if (self.parse_argv(goal_request.move_spec) < 0):
            # parsing args failed - return abort
            self.get_logger().error('action failed parsing move_spec')
            return GoalResponse.REJECT
        else:
            return GoalResponse.ACCEPT

    # call the subclass execute callback
    def call_exec_cb(self, goal_handle):
        self.get_logger().info('{} action_exec_cb called'.format(self.action_name))
        self.print()
        self._goal_handle = goal_handle
        move_results = []

        # set commanded linear/angular from current linear/angular to pick up current vel from which to slew
        self.commandedLinear = self.odom.twist.twist.linear.x
        self.commandedAngular = self.odom.twist.twist.angular.z

        # set up to loop on run() and send periodic feedback
        loop_period = 0.1
        feedback_period = 10    # give feedback every this-many loops
        loop_count = 0

        # call the run callback in a loop to perform the move
        try:
            while (rclpy.ok()):
                if self.run():
                    print("calling self.run")
                    break
                loop_count += 1
                if ((loop_count % feedback_period) == 0):
                    text_feedback, progress_feedback = self.get_feedback()
                    self.send_feedback(text_feedback, progress_feedback)
                time.sleep(loop_period)
        except Exception as e:
            print("Exception in call_exec_cb")
            self.get_logger().error(e)

        move_results = self.finish_cb()
        goal_handle.succeed()

        move_result = Move.Result()
        move_result.move_results = move_results
        self.get_logger().info('action finished with success, results: {}'.format(move_results))
        return move_result

    # start action server
    def create_action_server(self, action_name):
        self.action_name = action_name
        self.get_logger().info('starting action server for action {}'.format(action_name))
        self._goal_handle = None
        self._goal_lock = threading.Lock()
        self._action_server = ActionServer(
            self,
            Move,
            action_name,
            execute_callback=self.call_exec_cb,
            goal_callback=self.goal_callback,
            #handle_accepted_callback=self.handle_accepted_callback,
            #cancel_callback=self.cancel_callback,
            #callback_group=ReentrantCallbackGroup()
        )

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
