// Based on: https://raw.githubusercontent.com/ros/ros_tutorials/42f2cce5f098654a03301ae5e05aa88963990d64/turtlesim/tutorials/teleop_turtle_key.cpp
// Except changed to track velocity and publish changes, not commands; i.e. no
// new publish means last published Twist is still in effect.
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>

#define KEYCODE_R 0x43
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71

// Speed (MPH) [0,20]
#define STEP_SPEED 1
#define MAX_SPEED 20

// Steering (degrees) [0,10]
#define STEP_STEERING 1
#define MAX_STEERING 10

// 1 mph = 0.44704 m/s
#define MPH_IN_M_S 0.44704

// Simulate with smaller steering, max 100" turn radius
#define INCH_IN_M 0.0254
#define MIN_TURNING_RADIUS 100.0 * INCH_IN_M
double turnRadiusFromSteering(int steering) {
  // TODO: Research ackerman steering degrees to turn radius calculations
  // to make this better
  return MIN_TURNING_RADIUS * MAX_STEERING / steering;
}

class TeleopKey {
public:
  TeleopKey();
  void keyLoop();

private:
  ros::NodeHandle node;
  int speed, steering;
  double l_scale, a_scale;
  ros::Publisher twist_pub;
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
  speed = 0;
  steering = 0;

  l_scale = 1.0;
  a_scale = 1.0;
  node.param("scale_angular", a_scale, a_scale);
  node.param("scale_linear", l_scale, l_scale);

  twist_pub = node.advertise<geometry_msgs::Twist>("lisa/twist", 1);
}

void TeleopKey::keyLoop() {
  char c;
  bool dirty = false;

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
  speed = steering = 0;

  // Wait for keys to adjust those
  for(;;) {
    // get the next event from the keyboard
    if(read(kfd, &c, 1) < 0) {
      perror("read():");
      exit(-1);
    }
    ROS_DEBUG("value: 0x%02X\n", c);

    // Increase/decrease velocity by step according to key pressed
    switch(c) {
      case KEYCODE_L:
        ROS_DEBUG("LEFT");
        steering = std::min(MAX_STEERING, steering + STEP_STEERING);
        dirty = true;
        break;
      case KEYCODE_R:
        ROS_DEBUG("RIGHT");
        steering = std::max(-MAX_STEERING, steering - STEP_STEERING);
        dirty = true;
        break;
      case KEYCODE_U:
        ROS_DEBUG("UP");
        speed = std::min(MAX_SPEED, speed + STEP_SPEED);
        dirty = true;
        break;
      case KEYCODE_D:
        ROS_DEBUG("DOWN");
        speed = std::max(0, speed - STEP_SPEED);
        dirty = true;
        break;
    }

    // If dirty calculate linear and angular velocity and publish
    if(dirty == true) {
      ROS_INFO("speed=%d, steering=%d", speed, steering);
      geometry_msgs::Twist twist;
      // Calclate linear and angular velocity
      double linear = speed * MPH_IN_M_S;
      twist.linear.x = l_scale * linear;
      if (speed == 0) { // straight
        twist.angular.x = 0.0;
      } else { // turning
        // w = v / r
        double angular = linear / turnRadiusFromSteering(steering);
        twist.angular.x = a_scale * angular;
      }

      // Publish and clear dirty
      twist_pub.publish(twist);
      dirty = false;
    }
  }

  return;
}
