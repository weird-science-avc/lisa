#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/Empty.h>
#include <visualization_msgs/Marker.h>

ros::Publisher g_marker_pub;
ros::Publisher g_goal_pub;

#define MAX_GOALS 10
geometry_msgs::Pose g_goals[MAX_GOALS];
int g_goals_index = 0;
int g_goals_length = 0;

// TODO: Consider using Marker::POINTS for the waypoints instead, maybe easier
// to manage and clear?

bool clear_() {
  g_goals_length = g_goals_index = 0;
  // Remove all the markers
  visualization_msgs::Marker marker;
  marker.header.frame_id = "lisa";
  marker.header.stamp = ros::Time(); // 0 so it persists

  marker.ns = "waypoints";
  //marker.id = index;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::DELETE;

  for (int i = 0; i < MAX_GOALS; i++) {
    marker.id = i;
    g_marker_pub.publish(marker);
  }

  return true;
}

bool clear(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
  return clear_();
}

bool start_() {
  if (g_goals_length == 0) {
    ROS_ERROR("cannot start waypoint management, no waypoints defined");
    return false;
  }
  g_goals_index = 0;
  geometry_msgs::Pose goal = g_goals[g_goals_index];

  // Create internal goal message
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header.frame_id = "lisa";
  pose_stamped.header.seq = 0;
  pose_stamped.header.stamp = ros::Time::now();
  pose_stamped.pose = goal;
  g_goal_pub.publish(pose_stamped);
  ROS_INFO("START WAYPOINT 0: (%0.3f,%0.3f)", goal.position.x, goal.position.y);
  return true;
}

bool start(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
  return start_();
}

void goalCallback(const geometry_msgs::PoseStamped& msg) {
  if (g_goals_length >= MAX_GOALS) {
    ROS_ERROR("Dropped goal, alread at max");
    return;
  }

  // Save goal
  int index = g_goals_length++;
  g_goals[index] = msg.pose;

  // Publish marker for it
  visualization_msgs::Marker marker;
  marker.header.frame_id = "lisa";
  marker.header.stamp = ros::Time(); // 0 so it persists

  marker.ns = "waypoints";
  marker.id = index;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;

  marker.pose = msg.pose;
  marker.scale.x = 0.5;
  marker.scale.y = 0.5;
  marker.scale.z = 0.5;

  marker.color.a = 1.0;
  marker.color.g = 1.0;

  g_marker_pub.publish(marker);
  ROS_INFO("ADDED WAYPOINT %d: (%0.3f,%0.3f)", index, msg.pose.position.x, msg.pose.position.y);

  // And if its our first auto-start
  if (index == 0) {
    start_();
  }
}

void poseCallback(const geometry_msgs::PoseStamped& msg) {
  geometry_msgs::Pose g0;
  geometry_msgs::Vector3 v0;
  double d0;

  if (g_goals_length == 0) {
    ROS_DEBUG("no goals defined, skipping waypoint management");
    return;
  }

  if (g_goals_index >= g_goals_length) {
    ROS_DEBUG("reached final goal, skipping waypoint management");
    return;
  }

  // get our current goal
  g0 = g_goals[g_goals_index];

  // get a vector from the current pose to our goal (and distance)
  // TODO: Aren't there helpers for constructing the vector and getting distance, etc.?
  v0.x = g0.position.x - msg.pose.position.x;
  v0.y = g0.position.y - msg.pose.position.y;
  v0.z = g0.position.z - msg.pose.position.z;
  d0 = std::sqrt(std::pow(v0.x, 2.0) + std::pow(v0.y, 2.0) + std::pow(v0.z, 2.0));
  ROS_DEBUG("WAYPOINT %d: d=%0.3fm", g_goals_index, d0);

  // decide if we've arrived at it
  if (d0 < 0.1) { // Arrived at this goal
    ROS_INFO("FINISHED WAYPOINT %d: (%0.3f,%0.3f)", g_goals_index, g0.position.x, g0.position.y);
    g_goals_index++;

    // Detect finished
    if (g_goals_index >= g_goals_length) {
      ROS_INFO("FINISHED ALL WAYPOINTS");
      return;
    }

    // Update to next goal
    g0 = g_goals[g_goals_index];
    ROS_INFO("START WAYPOINT %d: (%0.3f,%0.3f)", g_goals_index, g0.position.x, g0.position.y);

    // Recalculate math
    v0.x = g0.position.x - msg.pose.position.x;
    v0.y = g0.position.y - msg.pose.position.y;
    v0.z = g0.position.z - msg.pose.position.z;
    d0= std::sqrt(std::pow(v0.x, 2.0) + std::pow(v0.y, 2.0) + std::pow(v0.z, 2.0));
  }

  // try to promote to next waypoint if we're closer to it than current
  // TODO: This was on an else in the other code, but seems like we should be
  // able to promote even if we've moved forward, it just probably wouldn't
  // happen.
  if (g_goals_index + 1 < g_goals_length) {
    geometry_msgs::Pose g1 = g_goals[g_goals_index + 1];
    geometry_msgs::Vector3 v1;
    double d1;
    v1.x = g1.position.x - msg.pose.position.x;
    v1.y = g1.position.y - msg.pose.position.y;
    v1.z = g1.position.z - msg.pose.position.z;
    d1 = std::sqrt(std::pow(v1.x, 2.0) + std::pow(v1.y, 2.0) + std::pow(v1.z, 2.0));
    if (d1 < d0) { // pose -> g1 smaller than pose -> g0, promote
      ROS_INFO("FINISHED WAYPOINT %d (PROMOTION): (%0.3f,%0.3f)", g_goals_index, g1.position.x, g1.position.y);
      g_goals_index++;
      ROS_INFO("START WAYPOINT %d: (%0.3f,%0.3f)", g_goals_index, g1.position.x, g1.position.y);
    }
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "waypoint_manager");
  ROS_INFO("waypoint_manager started");
  ros::NodeHandle n;

  // Setup publishers:
  // - visualization publisher
  g_marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 0);
  // - internal goal publisher
  g_goal_pub = n.advertise<geometry_msgs::PoseStamped>("lisa/goal", 1);

  // Setup subscribers:
  // - external goal publisher to accumulate waypoints from external sources
  ros::Subscriber goal_sub = n.subscribe("move_base_simple/goal", 1, goalCallback);
  // - localizer so we know where we're at and can move waypoints
  ros::Subscriber pose_sub = n.subscribe("lisa/pose", 1, poseCallback);

  // Setup services:
  ros::ServiceServer clear_service = n.advertiseService("/waypoint_manager/clear", clear);
  ros::ServiceServer start_service = n.advertiseService("/waypoint_manager/start", start);

  // Clear on start
  clear_();

  ros::spin();
  return 0;
}

//const float M_FT = 1.0 / 3.28;
//
//// *** DEFINE COURSE HERE ***
//const long EMERGENCY_TIMEOUT_MS = 2 * 60 * 1000;
//Waypoint waypoints[] = {
//  // Sparkfun AVC 2015
//  Waypoint{56.0 * M_FT, 0            , 5.0 * M_FT},   //Corner 1
//  //Waypoint{65.0 * M_FT, -142.5 * M_FT, 5.0 * M_FT},
//  Waypoint{70.0 * M_FT, -285.0 * M_FT, 5.0 * M_FT},   //Corner 2
////  Waypoint{10.0 * M_FT, -298.0 * M_FT, 5.0 * M_FT},  //          -8 drift y
//  Waypoint{-70.0 * M_FT, -303.0 * M_FT, 5.0 * M_FT}, // Corner 3 -8 drift y
//  //Waypoint{-69.0 * M_FT, -158.5 * M_FT, 5.0 * M_FT},
//  Waypoint{-78.0 * M_FT, -21.0 * M_FT, 5.0 * M_FT},    // Corner 4
//  Waypoint{10.0 * M_FT, -14.0 * M_FT, 5.0 * M_FT},
//  geometry_msgs::Point waypoints[] = {
//  geometry_msgs::Point(10.0, 0.0, 0.0)
//};

