#include <angles/angles.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Empty.h>
#include <stdio.h>
#include <tf/transform_datatypes.h>

#define PI 3.14159265

#define APPROACH_DELTA 1.0 // m

#define LOW_SPEED 0.3 // 0=stopped, 1=max
#define HIGH_SPEED 1.0 // 0=stopped, 1=max

// This is the range over which we'll do variable steering; anything greater
// than this will saturate at our maximum steering.
#define VARIABLE_STEERING_RANGE 0.52359 // 30 degrees

geometry_msgs::Pose g_pose, g_goal;
bool g_started;

void poseCallback(const geometry_msgs::PoseStamped& msg) {
  g_pose = msg.pose;
}

void goalCallback(const geometry_msgs::PoseStamped& msg) {
  g_goal = msg.pose;
  ROS_DEBUG("NAVIGATION: goal=(%0.3f,%0.3f)",
      g_goal.position.x, g_goal.position.y);
}

bool start(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
  ROS_INFO("NAVIGATION: STARTED");
  g_started = true;
  return true;
}

bool stop(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
  ROS_INFO("NAVIGATION: STOPPED");
  g_started = false;
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "navigator");
  ROS_INFO("navigator started");
  ros::NodeHandle n;

  // Setup publishers:
  ros::Publisher steering_pub = n.advertise<std_msgs::Float64>("lisa/cmd_steering", 1, true);
  ros::Publisher speed_pub = n.advertise<std_msgs::Float64>("lisa/cmd_speed", 1, true);

  // Setup subscribers:
  ros::Subscriber pose_sub = n.subscribe("/lisa/pose", 1, poseCallback);
  ros::Subscriber goal_sub = n.subscribe("/lisa/goal", 1, goalCallback);

  // Setup services:
  ros::ServiceServer start_service = n.advertiseService("/navigator/start", start);
  ros::ServiceServer stop_service = n.advertiseService("/navigator/stop", stop);

  // Loop variables
  double last_steering = 0.0;
  double last_speed = 0.0;
  ros::Rate loop_rate(10); // Hz
  while (ros::ok()) {
    // Get speed and steering based on started or stopped
    double steering = 0.0;
    double speed = 0.0;
    if (g_started) {
      // get a vector from the current pose to our goal (and distance)
      // TODO: Aren't there helpers for constructing the vector and getting distance, etc.?
      geometry_msgs::Vector3 v;
      v.x = g_goal.position.x - g_pose.position.x;
      v.y = g_goal.position.y - g_pose.position.y;
      v.z = g_goal.position.z - g_pose.position.z;
      double d = std::sqrt(std::pow(v.x, 2.0) + std::pow(v.y, 2.0) + std::pow(v.z, 2.0));
      double yaw = std::atan2(v.y, v.x); // [-PI, PI]
      double yaw_delta = angles::shortest_angular_distance(tf::getYaw(g_pose.orientation), yaw);
      ROS_DEBUG("NAVIGATION: pose=(%0.3f,%0.3f):%0.3f",
          g_pose.position.x, g_pose.position.y, tf::getYaw(g_pose.orientation));
      ROS_DEBUG("NAVIGATION: goal=(%0.3f,%0.3f)",
          g_goal.position.x, g_goal.position.y);
      ROS_DEBUG("NAVIGATION: v=(%0.3f,%0.3f), dist=%0.3f, yaw=%0.3f, yaw_delta=%0.3f",
          v.x, v.y, d, yaw, yaw_delta);

      // Discrete speed
      speed = (d > APPROACH_DELTA) ?  HIGH_SPEED : LOW_SPEED;

      // Continuous steering [-1.0, 1.0] within variable steering range
      steering = yaw_delta / VARIABLE_STEERING_RANGE;
      // If outside range, saturate to -1.0/1.0
      steering = std::min(1.0, std::max(-1.0, steering));
    }

    // Publish if changed
    if (steering != last_steering) {
      std_msgs::Float64 msg;
      msg.data = steering;
      steering_pub.publish(msg);
      ROS_INFO("NAVIGATION: steering=%0.3f", steering);
    }
    if (speed != last_speed) {
      std_msgs::Float64 msg;
      msg.data = speed;
      speed_pub.publish(msg);
      ROS_INFO("NAVIGATION: speed=%0.3f", speed);
    }

    // Loop variable
    last_steering = steering;
    last_speed = speed;

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
