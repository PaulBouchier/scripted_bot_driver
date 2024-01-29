#!/usr/bin/env python

import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from scripted_bot_driver.stop import Stop
from scripted_bot_driver.drive_straight_odom import DriveStraightOdom
from scripted_bot_driver.drive_waypoints import DriveWaypoints


loop_rate = 10       # loop rate

def usage():
    print('Usage: move_bot.py <command> [<parameters>]')
    print('Supported move commands are:')
    print('movo <distance> [speed] - drive straight for <distance> meters')
    print('arc <angle> <radius> <f | b> - drive an arc of <radius> subtending <angle> degrees forward or backward')
    print('roto <angle> [speed] - rotate <angle> degrees, +ve is CCW')
    print('drv_waypts <target_x> <target_y> [ more_targets ] - navigate to a list of targets')
    print('stop - ramp linear and rotational speed down to 0 with optional pause at end')
    sys.exit()

def main():
    if len(sys.argv) < 2:
        print('ERROR: No command was given')
        usage()
        sys.exit()

    rclpy.init()
    nh = None
    argv_index = 1

    match sys.argv[1]:
        case 'stop':
            nh = Stop()
        case 'movo':
            nh = DriveStraightOdom()
        case 'drive_waypoints':
            nh = DriveWaypoints()
        case _:
            print('Error: invalid command %s'%(sys.argv[1]))
            usage()
            sys.exit()

    argv_index = nh.parse_argv(sys.argv[2:])
    if (argv_index < 0):
        print('ERROR parsing args - exiting')
        sys.exit()

    nh.print()

    # set up the rate object that we sleep on
    loop_rate = 10       # loop rate
    nh.start_spin_thread()  # make rate work
    r = nh.create_rate(loop_rate)

    # pause until odometry is received
    while not nh.is_odom_started():
        time.sleep(1)
        nh.get_logger().info('slept on odometry')

    try:
        while (rclpy.ok()):
            # import pdb; pdb.set_trace()
            if nh.run():
                break
            r.sleep()

    except Exception as e:
        print(e)
        nh.get_logger().info("{} node terminated.".format(__file__))

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    nh.shutdown()     # nh should make things safe
    nh.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()