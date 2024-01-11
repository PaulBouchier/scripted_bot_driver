#!/usr/bin/env python

import sys
import time
import rospy
from geometry_msgs.msg import Twist
from mowbot_msgs.msg import OdomExtra, PlatformData
import tf
from math import radians, copysign, sqrt, pow, pi, asin, atan2
from MoveParent import MoveParent

loop_rate = 10       # loop rate

class DriveStraightOdom(MoveParent):
    def __init__(self, cmd_vel):
        super().__init__(cmd_vel)

    def parse_argv(self, argv):
        self.distance = float(argv[0])  # pick off first arg from supplied list
        self.odometer_goal = self.odom_extra.odometer + self.distance
        self.odometer_start = self.odom_extra.odometer

        # check whether a speed argument was supplied
        if len(argv) > 1:
            try:
                speed_arg = float(argv[1])
                self.speed = speed_arg
                print('Using supplied speed {}'.format(self.speed))
                return 2
            except ValueError:
                return 1
        return 1            # return number of args consumed

    def print(self):
        rospy.loginfo('Drive straight with odometry for {} m'.format(self.distance))

    # run is called at the rate until it returns true
    def run(self):
        if self.once:
            rospy.loginfo('start odometer: {}, goal: {}'.format(self.odom_extra.odometer, self.odometer_goal))
            self.once = False

        if ((self.distance >= 0 and self.odom_extra.odometer >= self.odometer_goal) or \
            (self.distance < 0 and self.odom_extra.odometer < self.odometer_goal)):
            rospy.loginfo('traveled: {} m'.format(self.odom_extra.odometer - self.odometer_start))
            return True

        # accelerate to full speed as long as we haven't reached the goal
        if self.distance >= 0:
            self.move_cmd.linear.x = self.slew_vel(self.speed)
        else:
            self.move_cmd.linear.x = self.slew_vel(-self.speed)

        rospy.loginfo(self.move_cmd.linear.x)
        self.cmd_vel.publish(self.move_cmd)
        return False

def shutdown():
    # Always stop the robot when shutting down the node.
    rospy.loginfo("Stopping the robot...")
    cmd_vel.publish(Twist())
    rospy.sleep(1)

def usage():
    print('Usage: drive_straight.py <distance> [speed] - drive the specified distance forward or backward, with optional speed')
    sys.exit()

if __name__ == '__main__':
    if len(sys.argv) != 2 and len(sys.argv) != 3:
        usage()
    argv_index = 1

    rospy.init_node('move', anonymous=False)
    rospy.on_shutdown(shutdown)     # Set rospy to execute a shutdown function when exiting

    global cmd_vel
    cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    r = rospy.Rate(loop_rate)

    try:
        m = DriveStraightOdom(cmd_vel)
        argv_index += m.parse_argv(sys.argv[argv_index:])
        m.print()
        while (not rospy.is_shutdown()):
            if m.run():
                break
            r.sleep()

    except Exception as e:
        print(e)
        rospy.loginfo("{} node terminated.".format(__file__))
