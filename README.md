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
* `/lisa/sensors/wheel_encoder`
  Wheel encoder data, always represents the latest. To publish:
  ```
  rostopic pub -1 /lisa/sensors/wheel_encoder std_msgs/UInt32 '20'
  ```

* `/lisa/sensors/imu`
  IMU data, always represents the latest. To publish:
  ```
  rostopic pub -1 /lisa/sensors/imu sensor_msgs/Imu '<todo-figure-out>'
  ```

#### Publishes
* `/lisa/pose`
  The pose (stamped) of the robot, i.e. its position and orientation. To follow:
  ```
  rostopic echo /lisa/pose
  ```

### Teleoperator (Keyboard)
```
rosrun lisa teleop_key
```

The Teleoperator (Keyboard) listens to the keyboard and publishes Twist
messages to control the robot.

#### Publishes
* `/lisa/twist`
  The Twist message with the target velocities the robot should attempt
  to achieve.
  ```
  rostopic echo /lisa/twist
  ```

### IMU Simulator
```
rosrun lisa imu_simulator
```

The IMU Simulator listens to Twist messages and generates IMU messages
according to the target twist message.

#### Subscribes
* `/lisa/twist`
  The Twist message with the target velocities the robot should attempt
  to achieve. To publish:
  ```
  rostopic pub -1 /lisa/twist geometry_msgs/Twist '<todo-figure-out>'
  ```

#### Publishes
* `/lisa/sensors/imu`
  The IMU values. To listen:
  ```
  rostopic echo /lisa/sensors/imu
  ```

### Wheel Encoder Simulator
```
rosrun lisa wheel_encoder_simulator
```

The Wheel Encoder Simulator listens to Twist messages and generates
wheel encoder messages according the target twist message.

#### Subscribes
* `/lisa/twist`
  The Twist message with the target velocities the robot should attempt
  to achieve. To publish:
  ```
  rostopic pub -1 /lisa/twist geometry_msgs/Twist '<todo-figure-out>'
  ```

#### Publishes
* `/lisa/sensors/wheel_encoder`
  The wheel encoder values. To listen:
  ```
  rostopic echo /lisa/sensors/wheel_encoder
  ```
