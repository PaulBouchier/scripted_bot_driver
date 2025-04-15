#!/usr/bin/env python

import sys
import time
import math

import rclpy
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Point

from scripted_bot_driver.move_parent import MoveParent

angular_control_zone_default = math.pi / 6.0  # modulate angular velocity according to error inside the control zone
dead_zone_default = math.pi / 60  # deadzone is +/- this for disablig angular rotation

class Seek2Can(MoveParent):
    def __init__(self):
        super().__init__('seek2can')

        self.declare_parameter('angular_control_zone_param', angular_control_zone_default)
        self.declare_parameter('dead_zone_param', dead_zone_default)

        self.angular_control_zone = self.get_parameter('angular_control_zone_param').value
        self.dead_zone = self.get_parameter('dead_zone_param').value

        self.get_logger().info('Seek2Can speed: {} rot_speed {}'.format(self.speed, self.rot_speed))
        self.range = 1.0
        self.bearing = 0.0

        self.start_action_server()

    def parse_argv(self, argv):
        self.run_once = True

        if (len(argv) != 0):
            self.get_logger().error('Incorrrect number of args given to seek2can: {}'.format(len(argv)))
            return -1

        self.target_sub = self.create_subscription(Point, 'closest_range_bearing', self.range_bearing_cb, 10)
        self.target_sub  # prevent unused variable warning

        return 0            # return number of args consumed

    def range_bearing_cb(self, msg):
        self.range = msg.x
        self.bearing = msg.y
        self.get_logger().debug('range: {} bearing {}'.format(self.range, self.bearing))

    def print(self):
        print('seek2can starting')

    # return True if motion is done
    def run(self):
        if not self.is_odom_started():
            self.get_logger().error('ERROR: robot odometry has not started - exiting')
            return True

        if self.run_once:
            self.run_once = False
            self.start_time = time.time()
            self.run_time = 0.0
            self.stopping = False

        # loop sending motion commands until we get close enough
        if self.range <= 0.1 or self.stopping:
            self.send_move_cmd(0.0, 0.0)
            return True

        # if we are not close enough, send motion commands
        if self.range > 0.1:
            speed = self.speed

        if abs(self.bearing) > self.angular_control_zone:
            rot_speed = self.rot_speed
        elif abs(self.bearing) < self.dead_zone:
            rot_speed = 0.0
        else:
            rot_speed = self.rot_speed * (abs(self.bearing) / self.angular_control_zone)
        if self.bearing < 0.0:
            rot_speed = -rot_speed

        self.send_move_cmd(self.slew_vel(speed), self.slew_rot(rot_speed))
        return False

    def start_action_server(self):
        self.create_action_server('seek2can')

    def get_feedback(self):
        progress = time.time() - self.start_time
        if progress > 10.0:     # don't let it run too long
            self.get_logger().info('seek2can: stopping due to timeout')
            self.send_move_cmd(self.slew_vel(0.0), self.slew_rot(0.0))
            self.stopping = True
        return 'range: {}'.format(self.range), progress

    def finish_cb(self):
        self.destroy_subscription(self.target_sub)
        self.target_sub = None
        self.get_logger().info('seek2can done')

        results = [self.range]
        return results

def main():
    rclpy.init()

    nh = Seek2Can()

    try:
        # Use a MultiThreadedExecutor to enable processing goals concurrently
        executor = MultiThreadedExecutor()
        rclpy.spin(nh, executor=executor)
    except KeyboardInterrupt:
        print("Shutting down after KeyboardInterrupt")

if __name__ == '__main__':
    main()