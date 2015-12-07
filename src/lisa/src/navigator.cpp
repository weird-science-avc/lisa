#include <angles/angles.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <std_srvs/Empty.h>
#include <stdio.h>
#include <tf/transform_datatypes.h>

#define PI 3.14159265

#define APPROACH_DELTA 1.0 // m

//#define LOW_VELOCITY 1.0 // m/s
//#define HIGH_VELOCITY 3.5 // m/s
#define LOW_VELOCITY 0.25 // m/s
#define HIGH_VELOCITY 1.00 // m/s

// angular = linear * yaw_delta * kAngularScale
#define HIGH_ANGULAR PI // rads/s
//const double kAngularScale = HIGH_ANGULAR / HIGH_VELOCITY / PI;
const double kAngularScale = HIGH_ANGULAR / HIGH_VELOCITY / PI / 5.0;

geometry_msgs::Pose g_pose, g_goal;
bool g_started;

void poseCallback(const geometry_msgs::PoseStamped& msg) {
  g_pose = msg.pose;
  //ROS_DEBUG("NAVIGATION: pose=(%0.3f,%0.3f):%0.3f",
  //    g_pose.position.x, g_pose.position.y, tf::getYaw(g_pose.orientation));
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
  ros::Publisher twist_pub = n.advertise<geometry_msgs::Twist>("lisa/twist", 1, true);

  // Setup subscribers:
  ros::Subscriber pose_sub = n.subscribe("/lisa/pose", 1, poseCallback);
  ros::Subscriber goal_sub = n.subscribe("/lisa/goal", 1, goalCallback);

  // Setup services:
  ros::ServiceServer start_service = n.advertiseService("/navigator/start", start);
  ros::ServiceServer stop_service = n.advertiseService("/navigator/stop", stop);

  // Loop variables
  geometry_msgs::Twist last_twist;
  ros::Rate loop_rate(10); // Hz
  while (ros::ok()) {
    // Get twisted based on started or stopped
    geometry_msgs::Twist twist;
    if (g_started) {
      // get a vector from the current pose to our goal (and distance)
      // TODO: Aren't there helpers for constructing the vector and getting distance, etc.?
      geometry_msgs::Vector3 v;
      v.x = g_goal.position.x - g_pose.position.x;
      v.y = g_goal.position.y - g_pose.position.y;
      v.z = g_goal.position.z - g_pose.position.z;
      double d = std::sqrt(std::pow(v.x, 2.0) + std::pow(v.y, 2.0) + std::pow(v.z, 2.0));
      double yaw = std::atan2(v.y, v.x); // [-PI, PI]
      float yaw_delta = angles::shortest_angular_distance(tf::getYaw(g_pose.orientation), yaw);
      ROS_DEBUG("NAVIGATION: pose=(%0.3f,%0.3f):%0.3f",
          g_pose.position.x, g_pose.position.y, tf::getYaw(g_pose.orientation));
      ROS_DEBUG("NAVIGATION: goal=(%0.3f,%0.3f)",
          g_goal.position.x, g_goal.position.y);
      ROS_DEBUG("NAVIGATION: v=(%0.3f,%0.3f), dist=%0.3f, yaw=%0.3f, yaw_delta=%0.3f",
          v.x, v.y, d, yaw, yaw_delta);

      // Compose a twist message for what to do with defaults
      twist.linear.x = LOW_VELOCITY;

      // Linear velocty is either high or low
      if (d > APPROACH_DELTA) {
        twist.linear.x = HIGH_VELOCITY;
      }

      // Angular velocty is continuous to max of 10deg left/right
      twist.angular.x = 0.0;

      if (std::abs(yaw_delta) > 0.0) {
        twist.angular.x = twist.linear.x * yaw_delta * kAngularScale;
      }
    }

    // Publish if changed
    if (twist.linear.x != last_twist.linear.x || twist.angular.x != last_twist.angular.x) {
      twist_pub.publish(twist);
      ROS_INFO("NAVIGATION: linear=%0.3f, angular=%0.3f", twist.linear.x, twist.angular.x);
    }

    // Loop variable
    last_twist = twist;

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
