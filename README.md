# lisa
[![Stories in Ready](https://badge.waffle.io/weird-science-avc/lisa.png?label=ready&title=Ready)](https://waffle.io/weird-science-avc/lisa)
[![CircleCI](https://circleci.com/gh/weird-science-avc/lisa.svg?style=svg)](https://circleci.com/gh/weird-science-avc/lisa)

This is the repository for the ROS based robot lisa, which is an
autonomous RC car controller.

## Pre-requisites

You either need to have a full ROS jade install, or you can use the
provided `Vagrantfile`. If you'd like to go that route, install
[VirtualBox](https://www.virtualbox.org/) and
[Vagrant](https://www.vagrantup.com/). Then you can do:

```
vagrant up
vagrant ssh
cd /vagrant # This folder is linked to the repo folder on your host
catkin_make
source devel/setup.bash
```

Then you can do normal ROS stuff from there. You probably want an X
server link to Vagrant installed so you can launch ROS's interactive
tools and they'll come up on your host. For example, with OSX you can
install [XQuartz](http://xquartz.macosforge.org/landing/) and then X
windows will come back to your host.

### Start roscore and log helpers
```
roscore
rosrun rqt_console rqt_console
rosrun rqt_logger_level rqt_logger_level
rosrun tf static_transform_publisher 0 0 0 0 0 0 "map" "lisa" 100
```

The `rqt_console` and `rqt_logger_level` command will popup the GUIs in
which you can set log levels and view ouptut. The static transform
publisher will just run but allows the various rviz integrations and
graphing to work.


## Nodes

### Automatic

You can start the core LISA nodes (i.e. roscore, localizer, waypoint
manager, etc.) by using roslaunch:

```
roslaunch core.launch
```

If you want to teleoperate, watch on rviz, simulate, etc. you'll need to
run specific nodes below.


### Localizer
```
rosrun lisa localizer
```

The localizer is responsible for tracing the robots position using
sensor data.

#### Subscribes
* `/lisa/sensors/wheel_encoder` - Wheel encoder data, always represents latest.
* `/lisa/sensors/imu` - IMU data, always represents the latest.

#### Publishes
* `/lisa/pose` - The pose (stamped) of the robot; i.e. its position and orientation.


### Navigator
```
rosrun lisa navigator
```

The navigator is responsible for listening to goals and poses and
sending command instructions for steering and velocity.

#### Subscribes
* `/lisa/pose` - The robots current pose.
* `/lisa/goal` - The robots current goal.

#### Publishes
* `/lisa/cmd_steering` - The target steering from [-1.0,1.0]; this is
  unitless and is meant to represent steer all left (1.0) and all right
  (-1.0) and continously in between.
* `/lisa/cmd_velocity` - The target velocity in m/s.

#### Services
* `/navigator/start` - Starts navigation of the robot according to the goal topic.
* `/navigator/stop` - Stops navigation and the robot..


### Waypoint Manager
```
rosrun lisa waypoint_manager
```

The waypoint manager is responsible for accumulating waypoints and
updating LISA's internal goal based on the next waypoint.

#### Subscribes
* `/move_base_simple/goal` - Adds a goal (in-order) as a waypoint; can
  be used with RViz '2D Nav Goal'.
* `/lisa/pose` - The pose (stamped) of the robot; i.e. its position and orientation.

#### Publishes
* `/visualization_marker` - The goals/waypoints received as SPHERE markers.
* `/lisa/goal` - The goals/waypoints received as SPHERE markers.

#### Services
* `/waypoint_manager/clear` - Clears any current waypoints.
* `/waypoint_manager/reset` - Resets to the first waypoint.


### Teleoperator (Keyboard)
```
rosrun lisa teleop_key
```

The Teleoperator (Keyboard) listens to the keyboard and publishes
steering and velocity messages.

#### Publishes
* `/lisa/cmd_steering` - The target steering from [-1.0,1.0]; this is
  unitless and is meant to represent steer all left (1.0) and all right
  (-1.0) and continously in between.
* `/lisa/cmd_velocity` - The target velocity in m/s.


### IMU Simulator
```
rosrun lisa imu_simulator
```

The IMU Simulator listens to steering and velocity messages and
generates IMU messages that simulate the robots behavior.

#### Subscribes
* `/lisa/cmd_steering` - The target steering from [-1.0,1.0]; this is
  unitless and is meant to represent steer all left (1.0) and all right
  (-1.0) and continously in between.
* `/lisa/cmd_velocity` - The target velocity in m/s.

#### Publishes
* `/lisa/sensors/imu` - The IMU values.


### Wheel Encoder Simulator
```
rosrun lisa wheel_encoder_simulator
```

The Wheel Encoder Simulator listens to velocity messages and generates
wheel encoder messages that simulate the robots behavior.

#### Subscribes
* `/lisa/cmd_velocity` - The target velocity in m/s.

#### Publishes
* `/lisa/sensors/wheel_encoder` - The wheel encoder values.


### Visualizer
```
rosrun rviz rviz --display-config default.rviz
```

This starts rviz with the proscribed configuration. **Changes to the
running app won't be saved to the config unless you manually save
them.**


## Debug

To listen to any publshed topic above use `rostopic echo <topic>`; for
example to listen to pose:
```
rostopic echo /lisa/pose
```

In order to publish fake data on sensors, commands, etc. you can directly publish to the topics using `rostopic pub ...`.

### Wheel Encoder
```
rostopic pub -1 /lisa/sensors/wheel_encoder std_msgs/UInt32 '<ticks>'
```
The `<ticks>` value should be the wheel encoder ticks value.

### IMU
```
rostopic pub -1 /lisa/sensors/imu sensor_msgs/Imu -- '[0, [0,0], "lisa"]' '<orientation>' '<orientation_covariance>' '<angular>' '<angular_covariance>' '<linear>' '<linear_covariance>'
```
* `<orientation>` is a quaternion and should be an array of four doubles `[x, y, z, w]`.
* `<angular>` is a vector3 for angular velocity and array of three doubles `[x, y, z]`.
* `<linear>` is a vector3 for linear velocity and array of three doubles `[x, y, z]`.
* `<*_covariance>` is an array of nine doubles; should be all zeros for unknown.

A quaternion can be computed using http://quat.zachbennett.com/; put
degrees in for `Z` and then `z = <q1>` and `w = <q4>`.

For example for PI/2 (90 deg) rotation:
```
rostopic pub -1 /lisa/sensors/imu sensor_msgs/Imu -- '[0, [0,0], "lisa"]' '[0.0, 0.0, 0.707, 0.707]' '[0,0,0,0,0,0,0,0,0]' '[0,0,0]' '[0,0,0,0,0,0,0,0,0]' '[0,0,0]' '[0,0,0,0,0,0,0,0,0]'
```

### Robot Commands
#### Start and Stop
```
rosservice call /navigator/start
rosservice call /navigator/stop
```
#### Move
```
rostopic pub -1 /lisa/cmd_steering st_msgs/Float64 '<steering>'
rostopic pub -1 /lisa/cmd_velocity st_msgs/Float64 '<velocity>'
```
* `<steering>` is a unitless value within [-1.0,1.0] to represent the
  amount of steering from all left (1.0) to all right (-1.0).
* `<velocity>` is the target velocity in m/s.

For example for 1 m/s turning a litle to the left:
```
rostopic pub -1 /lisa/cmd_steering st_msgs/Float64 '0.25'
rostopic pub -1 /lisa/cmd_velocity st_msgs/Float64 '1.0'
```

### Waypoints

To add a waypoint to the current list
```
rostopic pub -1 /lisa/sensors/imu sensor_msgs/Imu -- '[0, [0,0], "lisa"]' '<position>' '<orientation>'
```
* `<position>` is a 3d point and should be an array of three doubles `[x, y, z]`.
* `<orientation>` is a quaternion and should be an array of four doubles `[x, y, z, w]`.
**NOTE: The waypoint only pays attention to `position.x` and
`position.y` currently.**
