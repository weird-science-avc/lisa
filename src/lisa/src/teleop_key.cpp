// Based on: https://raw.githubusercontent.com/ros/ros_tutorials/42f2cce5f098654a03301ae5e05aa88963990d64/turtlesim/tutorials/teleop_turtle_key.cpp
// Except changed to track speed/steering and publish changes, not commands; i.e. no
// new publish means last published Twist is still in effect.
#include <ros/ros.h>
#include <signal.h>
#include <std_msgs/Float64.h>
#include <stdio.h>
#include <termios.h>

#define KEYCODE_R 0x43
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71

// Speed [0, 1.0]
#define STEP_SPEED 0.01
#define MIN_SPEED 0.0
#define MAX_SPEED 1.0

// Steering [1.0, -1.0]
#define STEP_STEERING 0.05
#define MAX_STEERING 1.0
#define MIN_STEERING -1.0

class TeleopKey {
public:
  TeleopKey();
  void keyLoop();

private:
  ros::NodeHandle node;
  double speed, steering;
  double l_scale, a_scale;
  ros::Publisher steering_pub;
  ros::Publisher speed_pub;
};

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_key");
  ROS_INFO("teleop_key started");
  TeleopKey teleop_key;

  signal(SIGINT, quit);

  teleop_key.keyLoop();

  return(0);
}

TeleopKey::TeleopKey() {
  speed = 0.0;
  steering = 0.0;

  l_scale = 1.0;
  a_scale = 1.0;
  node.param("scale_angular", a_scale, a_scale);
  node.param("scale_linear", l_scale, l_scale);

  steering_pub = node.advertise<std_msgs::Float64>("lisa/cmd_steering", 1, true);
  speed_pub = node.advertise<std_msgs::Float64>("lisa/cmd_speed", 1, true);
}

void TeleopKey::keyLoop() {
  char c;

  // get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to adjust speed and steering.");

  // initialize speed and steering
  speed = steering = 0.0;

  // Wait for keys to adjust those
  for(;;) {
    // get the next event from the keyboard
    if(read(kfd, &c, 1) < 0) {
      perror("read():");
      exit(-1);
    }
    ROS_DEBUG("value: 0x%02X\n", c);

    // Increase/decrease by step provided, then normalize
    double newSteering = steering;
    double newSpeed = speed;
    switch(c) {
      case KEYCODE_L:
        ROS_DEBUG("LEFT");
        newSteering = newSteering + STEP_STEERING;
        break;
      case KEYCODE_R:
        ROS_DEBUG("RIGHT");
        newSteering = newSteering - STEP_STEERING;
        break;
      case KEYCODE_U:
        ROS_DEBUG("UP");
        newSpeed = newSpeed + STEP_SPEED;
        break;
      case KEYCODE_D:
        ROS_DEBUG("DOWN");
        newSpeed = newSpeed - STEP_SPEED;
        break;
    }
    // Normalize into valid range
    newSteering = std::min(MAX_STEERING, std::max(MIN_STEERING, newSteering));
    newSpeed = std::min(MAX_SPEED, std::max(MIN_SPEED, newSpeed));

    // If either dirty log and publish
    if(newSteering != steering || newSpeed != speed) {
      // Assign to class
      steering = newSteering;
      speed = newSpeed;

      // Print message
      std_msgs::Float64 msg;
      ROS_INFO("speed=%0.2f, steering=%0.2f", speed, steering);

      // Publish steering and speed
      msg.data = steering;
      steering_pub.publish(msg);
      msg.data = speed;
      speed_pub.publish(msg);
    }
  }

  return;
}
