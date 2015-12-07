# lisa

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
```

The `rqt_console` and `rqt_logger_level` command will popup the GUIs in
which you can set log levels and view ouptut.


## Nodes


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


### Teleoperator (Keyboard)
```
rosrun lisa teleop_key
```

The Teleoperator (Keyboard) listens to the keyboard and publishes Twist
messages to control the robot.

#### Publishes
* `/lisa/twist` - The Twist message with the target velocities.


### IMU Simulator
```
rosrun lisa imu_simulator
```

The IMU Simulator listens to Twist messages and generates IMU messages
according to the target twist message.

#### Subscribes
* `/lisa/twist` - The Twist message with the target velocities.

#### Publishes
* `/lisa/sensors/imu` - The IMU values.


### Wheel Encoder Simulator
```
rosrun lisa wheel_encoder_simulator
```

The Wheel Encoder Simulator listens to Twist messages and generates
wheel encoder messages according the target twist message.

#### Subscribes
* `/lisa/twist` - The Twist message with the target velocities.

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

In order to publish fake data on sensors, twist, etc. you can directly publish to the topics using `rostopic pub ...`.

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

### Robot Target Velocities
```
rostopic pub -1 /lisa/twist geometry_msgs/Twist -- '<linear>' '<angular>'
```
* `<linear>` is a vector3 for linear velocity and array of three doubles
  `[x, y, z]`; use only `x` for velocity.
* `<angular>` is a vector3 for angular velocity and array of three
  doubles `[x, y, z]`; use only `x` for velocity.

For example for 1 m/s straight forward, turning 5 degrees per second:
```
rostopic pub -1 /lisa/twist geometry_msgs/Twist -- '[1.0, 0.0, 0.0]' '[0.0872665, 0.0, 0.0]'
```
