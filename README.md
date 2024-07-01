# scripted_bot_driver
Scripted_bot_driver is a set of nodes which outputs motion commands and monitors odometry feedback to cause the robot to move according to provided commands and parameters

The nodes are designed to run a robot which accepts motion commands from Twist
messages sent on the /cmd_vel topic, and which provides pose feedback in Odometry
messages sent on the /odom topic. The nodes use these fields from the Odometry message:
- pose.pose.position.x & y - the x-y position of the robot
- pose.pose.orientation.x & y & z & w - Quaternion describing the robot's heading
- twist.twist.linear.x - the linear speed of the robot
- twist.twist.angular.z - the angular speed of the robot

At the top of the stack is the scripted_bot_driver node, which
interprets command-line verbs to drive straight (movo), or traverse waypoints
(drive_wayponts) or stop, each of which may have multiple parameters. It
sequences each verb with its parameters out to action servers. The action servers perform
the actions by publishing Twist messages on cmd_vel until the action is complete.
They signal action complete, which causes scripted_bot_driver to interpret the next in a string
of verbs and sequence it to the appropriate action server, or exit if there are no more verbs.

The action servers must be started before scripted_mover requests any actions:
```
ros2 launch scripted_bot_driver servers_launch.py
```

Run scripted_bot_driver without any commands to get the usage message:
```
Usage: scripted_mover.py [commands] - executes the series of move commands provided
Supported move commands are:
movo <distance> [speed] - drive straight for <distance> meters
roto <target_angle>[d|p] <mode> [angular_speed][d|p] [drive_speed]
drive_waypoints <target_x> <target_y> [ more_targets ] - drive to a list of targets
stop [delay]- ramp linear and rotational speed down to 0 with optional pause at end
```

An example of a command to run the Quick Trip DPRG contest could be:
```
ros2 run scripted_bot_driver scripted_mover movo 3.2 stop 2 movo -3.2
```
and a command to run the Four Corners DPRG contest could be:
```
ros2 run scripted_bot_driver scripted_mover drive_waypoints 3 0 3 3 0 3 0 0
```
Distance parameters are in meters and delay is in seconds.


roto and movo could also be combined to run Four Corners:
```
ros2 run scripted_bot_driver scripted_mover movo 3 roto 90d movo 3 roto 90d movo 3 roto 90d movo 3 roto 90d
```
roto is used to turn in relation to the current heading: 
- Unmodified target angles can be specified as just a number in radians with positive values requesting a CCW direction. If a 'd' or 'p' is appended, the angle will be interpreted as degrees, or to be multiplied by pi, respectively.
- so -3.14159 and -1p and -180d are all treated as a half rotation in the clockwise direction in strict mode.
- mode is either 1 or 2 if specified. Mode 1 is the default and seeks the shortest path. So if not specified, a roto 181d would actually result in a -179 degree rotation. The specified target angle will be normalized, so you'll never see more than a half rotation. Mode 2 is strict, meaning it requests a relative rotation from the starting heading exactly as specified. A "roto -4p 2" will execute 2 full rotations clockwise.
- the remaining optional arguments require all preceding arguments to be supplied.
- angular_speed, will override the default turning speed with the supplied speed in radians.
- drive_speed, will add a forward or backward velocity, resulting in arcs. The value is in meters per second. Vary turning speed and drive speed for varying arcs. The terminating condition is always the final target rotation angle.
- Accuracy is currently quite low and will always over-rotate. Reduce the angular_velocity to achieve better results. 

A compatible simulation can be launched from the generic_turtlesim package, which
is built around the ros2 turtlesim simulatork. It just reformats the simulator pose output
into Odometry messages on /odom.
```
ros2 launch generic_turtlesim generic_turtlesim_launch.yaml
```
