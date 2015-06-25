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

## Localizer

The localizer is responsible for tracing the robots position using
sensor data. To start it after sourcing the `devel/setup.bash` you can
do (each command in a separate terminal or screen):

```
roscore
rosrun rqt_console rqt_console
rosrun rqt_logger_level rqt_logger_level
rosrun lisa localizer
```

The `rqt_console` and `rqt_logger_level` command will popup the GUIs in
which you can set log levels and view ouptut.  Then you can publish to
the topics `/lisa/sensors/imu` or `/lisa/sensors/wheel_encoder` to send
it information which to update the position:

```
rostopic pub -1 /lisa/sensors/wheel_encoder std_msgs/UInt32 '20'
```
__TODO: Not sure how to publish `sensor_msgs/Imu` yet__

Those may show up on `rqt_console` if you have debug logging
enabled. Otherwise you can echo the topic `/lisa/pose` to see odometry
updates.
