#!/usr/bin/env python

import sys
import time
from math import sqrt, pow, pi, atan2

import rclpy
import tf_transformations

from scripted_bot_driver.move_parent import MoveParent
from scripted_bot_interfaces.msg import WaypointsDebug

err_circle = 1.0    # meters, distance within which we consider goal achieved
dead_zone = pi / 40  # deadzone is +/- this for disablig angular rotation
downramp = 0.75       # downramp is distance at which speed is reduced to slow

class TargetXY():
    def __init__(self, target_x, target_y):
        self.target_x = target_x
        self.target_y = target_y
    def get_xy(self):
        return self.target_x, self.target_y

class DriveWaypoints(MoveParent):
    def __init__(self):
        super().__init__('drive_waypoints')

        self.debug_msg = WaypointsDebug()
        self.debug_pub = self.create_publisher(WaypointsDebug, 'waypoints_debug', 10)

    def parse_argv(self, argv):
        self.get_logger().info('parsing move_spec {}'.format(argv))
        self.distance = 0.0
        self.bearing = 0.0
        self.stopping = False
        self.run_once = True
        num_points = 0
        self.target_list = []
        self.set_defaults()

        if (len(argv) < 2):
            self.get_logger().fatal('Incorrrect number of args given to DriveWaypoints: {}'.format(len(argv)))
            return -1

        while (len(argv) >= (num_points + 2)):
            try:
                target_x = float(argv[num_points])  # pick off first args from supplied list
                target_y = float(argv[num_points+1])
                t = TargetXY(target_x, target_y)
                self.target_list.append(t)
                num_points += 2
            except ValueError:
                break

        self.get_logger().info('DriveWaypoints parsed {} points'.format(num_points))
        self.current_target = 0     # position in the list of targets
        return num_points           # return number of args consumed

    def print(self):
        target_string = ''
        for t in self.target_list:
            x, y = t.get_xy()
            target_string += '[{}, {}], '.format(x, y)
        self.get_logger().info('Navigate to: {}'.format(target_string))

    # run is called at the rate until it returns true
    def run(self):
        if not self.is_odom_started():
            self.get_logger().fatal('ERROR: robot odometry has not started - exiting')
            return True

        if self.current_target == len(self.target_list):
            self.get_logger().fatal('DriveWaypoints.run() called with empty list - exiting')
            return True         # done

        target_x, target_y = self.target_list[self.current_target].get_xy()

        if self.run_once:
            at_target, self.distance, self.bearing = self.target_vector(target_x, target_y, self.distance)

            self.get_logger().info('Start driving to: [{}, {}] distance: {:.02f} bearing: {:.02f}, at_target: {}'.format(
                target_x, target_y, self.distance, self.bearing, at_target))
            self.initial_distance = self.distance
            self.run_once = False

        if self.navigate_target(target_x, target_y):
            # at current_target, set current_target to next in list
            self.current_target += 1
            if self.current_target == len(self.target_list):
                self.get_logger().info('DriveWaypoints done with target list')
                return True         # done
            target_x, target_y = self.target_list[self.current_target].get_xy()
            self.distance = 0
            self.run_once = True

            # rotate to the heading that points at the next target
            at_target, self.distance, self.bearing = self.target_vector(target_x, target_y, self.distance)
            bearing_deg = '{:.02f}'.format(self.bearing * (180 / pi))
            return False

        return False

    def navigate_target(self, target_x, target_y):
        speed = 0.0
        angular = 0.0

        at_target, self.distance, self.bearing = self.target_vector(target_x, target_y, self.distance)
        if self.stopping:
            if self.odom.twist.twist.linear.x < 0.01 and self.odom.twist.twist.angular.z < 0.01:
                self.stopping = False
                return True
            else:
                return False        # wait for robot to stop
        if at_target:
            self.get_logger().info('navigate_target found it is at target, stopping')
            self.send_move_cmd(self.slew_vel(0.0), self.slew_rot(0.0))
            self.stopping = True
            return False

        # turn toward target if needed. Don't turn if within the error circle
        if self.bearing < dead_zone and self.bearing > -dead_zone:
            angular = self.slew_rot(0.0)
        else:
            if self.bearing > dead_zone and self.distance > err_circle:
                angular = self.slew_rot(self.rot_speed)
            if self.bearing < -dead_zone and self.distance > err_circle:
                angular = self.slew_rot(-self.rot_speed)

        # set linear speed, slow down on approach
        if self.distance < downramp:
            speed = self.speed * (self.distance / downramp)
            if speed < self.low_speed:
                speed = self.low_speed
            if speed > self.speed:
                speed = self.speed
        else:
            speed = self.speed

        self.get_logger().debug('navigate_target set speed to linear: {:.02f} angular: {:.02f}'.format(
            speed, angular))
        self.send_move_cmd(self.slew_vel(speed), self.slew_rot(angular))

    def target_vector(self, target_x, target_y, distance):
        self.get_logger().debug('Entered target_vector({}, {}, {})'.format(
            target_x, target_y, distance))
        last_distance = distance
        x_dist = target_x - self.odom.pose.pose.position.x
        y_dist = target_y - self.odom.pose.pose.position.y
        distance = sqrt(pow(x_dist, 2) + pow(y_dist, 2))

        # compute heading from /odom
        q = [
            self.odom.pose.pose.orientation.x,
            self.odom.pose.pose.orientation.y,
            self.odom.pose.pose.orientation.z,
            self.odom.pose.pose.orientation.w,
        ]
        euler_angles = tf_transformations.euler_from_quaternion(q)
        heading = self.normalize(euler_angles[2])

        # from dpa page: target_angle = (90 - (atan2(yd,xd)*(180/PI))) - (heading*(180/PI));
        bearing = atan2(y_dist, x_dist) - heading
        bearing_normalized = self.normalize(bearing)
        at_target = self.target_acquired(distance, last_distance)

        # publish debug data
        self.get_logger().debug(
            'target_vector: distance: {:.02f} heading: {:.02f} bearing: {:.02f} normalized bearing: {:.02f}'.format(
            distance, heading, bearing, bearing_normalized))
        self.debug_msg.target_x = target_x
        self.debug_msg.target_y = target_y
        self.debug_msg.at_target = at_target
        self.debug_msg.x_distance = x_dist
        self.debug_msg.y_distance = y_dist
        self.debug_msg.distance = distance
        self.debug_msg.bearing = bearing
        self.debug_msg.bearing_normalized = bearing_normalized
        self.debug_pub.publish(self.debug_msg)


        return at_target, distance, bearing_normalized

    def target_acquired(self, distance, last_distance):
        self.get_logger().debug('Entered target_acquired({}, {}'.format(distance, last_distance))
        if (distance < err_circle and distance > last_distance): 
            return True
        else:
            return False

    def normalize(self, angle):     # normalize angle to +/- pi
        if angle > pi:
            angle -= 2 * pi
        if angle < -pi:
            angle += 2 * pi
        return angle

    def start_action_server(self):
        self.create_action_server('drive_waypoints')

    def get_feedback(self):
        target_x, target_y = self.target_list[self.current_target].get_xy()
        text_feedback = 'Driving to {} {}, progress %: '.format(
            target_x, target_y)
        progress_feedback = ((self.initial_distance - self.distance) / self.initial_distance) * 100
        return text_feedback, progress_feedback

    def finish_cb(self):
        results = [self.distance]
        return results

def main():
    rclpy.init()
    nh = DriveWaypoints()
    nh.start_action_server()
    nh.start_spin_thread()

    rclpy.spin(nh)

if __name__ == '__main__':
    main()
