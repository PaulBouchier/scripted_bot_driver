#!/usr/bin/env python

import sys
import time
from math import pow, sqrt, pi, atan2

import rclpy
from rclpy.executors import MultiThreadedExecutor

from scripted_bot_driver.move_parent import MoveParent
from scripted_bot_interfaces.msg import WaypointsDebug
from robot_interfaces.msg import PlatformData

'''
The Seek2Cone class drives toward where it detects the cone, for a limited distance. It completes
the action when it either detects hitting the cone (bumper pressed), or max distance reached.
This class is derived from drive_waypoints
'''

err_circle = 1.0    # meters, distance within which we consider goal achieved
dead_zone = pi / 90  # deadzone is +/- this for disablig angular rotation
angular_control_zone = pi / 6.0  # modulate angular velocity according to error inside the control zone
downramp = 0.75       # downramp is distance at which speed is reduced to slow

class TargetXY():
    def __init__(self, target_x, target_y):
        self.target_x = target_x
        self.target_y = target_y
    def get_xy(self):
        return self.target_x, self.target_y

class Seek2Cone(MoveParent):
    def __init__(self):
        super().__init__('seek2cone')

        self.bumper_pressed = False

        # subscriber to robot PlatformData
        self.subscription = self.create_subscription(
            PlatformData, 'platform_data', self.platform_data_cb, 10)
        
        # Publisher for debug data
        self.debug_msg = WaypointsDebug()
        self.debug_pub = self.create_publisher(WaypointsDebug, 'seek2cone_debug', 10)

        # create the action server for this move-type
        self.create_action_server('seek2cone')

    def platform_data_cb(self, msg):
        if (not self.bumper_pressed and msg.bumper_pressed):
            self.bumper_pressed = True  # latch bumper_pressed
            self.get_logger().info('Detected bumper pressed!!!')

    def parse_argv(self, argv):
        self.get_logger().info('parsing move_spec {}'.format(argv))
        self.run_once = True
        num_points = 0
        self.target_list = []
        self.set_defaults()


        if (len(argv) < 2):
            self.get_logger().fatal('Incorrect number of args given to Seek2Cone: {}'.format(len(argv)))
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

        self.get_logger().info('Seek2Cone parsed {} points'.format(num_points))
        self.current_target = 0     # position in the list of targets
        return num_points           # return number of args consumed

    # run is called at the rate until it returns true. It does not stop motion on
    # completion - caller is responsible for stopping motion.
    def run(self):
        if not self.is_map_started() or not self.is_odom_started():
            self.get_logger().fatal('ERROR: robot odom or map topic has not started - exiting')
            return True

        target_x, target_y = self.target_list[self.current_target].get_xy()
        
        if self.run_once:
            self.bumper_pressed = False
            self.distance = 0.0         # distance to cone waypoint
            self.bearing = 0.0          # bearing to cone waypoint
            self.stopping = False
            self.delta_odom = 0.0       # how far we've traveled seeking cone
            self.initial_x = self.odom.pose.pose.position.x
            self.initial_y = self.odom.pose.pose.position.y

            # set this False to disable back_and_aim()
            self.aim = True  
 
            at_target, self.distance, self.bearing = self.target_vector(target_x, target_y, self.distance)

            self.get_logger().info('Seek cone at: [{}, {}] distance: {:.02f} bearing: {:.02f}, at_target: {}'.format(
                target_x, target_y, self.distance, self.bearing, at_target))
            self.initial_distance = self.distance
            self.initial_bearing = self.bearing
            self.run_once = False

        # Check end conditions
        if (self.bumper_pressed or self.delta_odom > self.distance+1.5):
            self.send_move_cmd(0.0, 0.0)  # hit cone or traveled max distance, slam on the brakes
            self.get_logger().info('traveled: {} m'.format(self.delta_odom))
            return True

        if (self.aim):
            if (self.back_and_aim(target_x, target_y)):  # initially, back up and aim at the cone. True return when aimed
                self.aim = False
        else:
            if (self.navigate_target(target_x, target_y)):
                return True     # if we reach target and no bumper and haven't exceeded max dist, call it done

        return False

    def back_and_aim(self, target_x, target_y):
        speed = -self.speed  # drive backwards while rotating to aim at cone
        angular = 0.0

        at_target, self.distance, self.bearing = self.target_vector(target_x, target_y, self.distance)

        self.get_logger().info(self.distance, self.initial_distance, self.bearing, self.initial_bearing)

        if ((self.initial_bearing < 0.0 and self.bearing > 0.0 or self.initial_bearing > 0.0 and self.bearing < 0.0)
            and (abs(self.distance - self.initial_distance) > 1.0)):
            self.aim = False  # bearing changed sign from initial bearing, so we should be pointed straight at cone
            self.send_move_cmd(0.0, 0.0)  # jam on the brakes, done with back_and_aim
            return True
        else:
            if (self.bearing > 0.0):
                angular = self.low_rot_speed
            else:
                angular = -self.low_rot_speed

        self.get_logger().debug('back_and_aim() set speed to linear: {:.02f} angular: {:.02f}'.format(
            speed, angular))
        self.send_move_cmd(self.slew_vel(speed), self.slew_rot(angular))
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
        if (self.bearing < dead_zone and self.bearing > -dead_zone) or (self.distance < err_circle):
            angular = self.slew_rot(0.0)
        else:
            if (self.bearing > angular_control_zone):
                angular = self.rot_speed
            elif (self.bearing > dead_zone):
                angular = self.low_rot_speed
            elif (self.bearing < -angular_control_zone):
                angular = -self.rot_speed
            elif (self.bearing < -dead_zone):
                angular = -self.low_rot_speed

        # set linear speed, slow down on approach
        if self.distance < downramp:
            speed = self.speed * (self.distance / downramp)
            if speed < self.low_speed:
                speed = self.low_speed
            if speed > self.speed:
                speed = self.speed
        else:
            speed = self.speed

        self.get_logger().debug('navigate_target() set speed to linear: {:.02f} angular: {:.02f}'.format(
            speed, angular))
        self.send_move_cmd(self.slew_vel(speed), self.slew_rot(angular))
        return False

    def target_vector(self, target_x, target_y, distance):
        self.get_logger().debug('Entered target_vector({}, {}, {})'.format(
            target_x, target_y, distance))
        last_distance = distance
        x_dist = target_x - self.map.pose.position.x
        y_dist = target_y - self.map.pose.position.y
        distance = sqrt(pow(x_dist, 2) + pow(y_dist, 2))

        # compute heading from /map
        euler_angles = self.euler_from_quaternion(self.map.pose.orientation)
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
    nh = Seek2Cone()

    try:
        # Use a MultiThreadedExecutor to enable processing goals concurrently
        executor = MultiThreadedExecutor()
        rclpy.spin(nh, executor=executor)
    except KeyboardInterrupt:
        print("Shutting down after KeyboardInterrupt")

if __name__ == '__main__':
    main()
