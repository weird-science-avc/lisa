#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/Empty.h>
#include <visualization_msgs/Marker.h>

#define TOLERANCE 0.5

ros::Publisher g_marker_pub;
ros::Publisher g_goal_pub;

#define MAX_GOALS 10
geometry_msgs::Pose g_goals[MAX_GOALS];
int g_goal_statuses[MAX_GOALS]; // 0=NOT_REACHED, 1=REACHED, 2=REACHED_PROMOTION
int g_goals_index = 0;
int g_goals_length = 0;

visualization_msgs::Marker createMarker() {
  visualization_msgs::Marker marker;
  marker.header.frame_id = "lisa";
  marker.header.stamp = ros::Time();

  marker.ns = "waypoints";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE_LIST;
  marker.action = visualization_msgs::Marker::ADD;

  marker.scale.x = TOLERANCE;
  marker.scale.y = TOLERANCE;
  marker.scale.z = TOLERANCE;

  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;

  for (int i = 0; i < g_goals_length; i++) {
    marker.points.push_back(g_goals[i].position);
    std_msgs::ColorRGBA color;
    color.a = 1.0;
    switch(g_goal_statuses[i]) {
      case 2:
        color.r = 0.5;
        color.g = 0.5;
        color.b = 0.0;
        break;
      case 1:
        color.r = 0.0;
        color.g = 1.0;
        color.b = 0.0;
        break;
      default:
        color.r = 1.0;
        color.g = 1.0;
        color.b = 0.0;
        break;
    }
    marker.colors.push_back(color);
  }

  return marker;
}

// TODO: Consider using Marker::POINTS for the waypoints instead, maybe easier
// to manage and clear?
      //path_strip.points.push_back(g_pose.pose.position);

bool clear_() {
  g_goals_length = g_goals_index = 0;

  // Publish an updated marker
  visualization_msgs::Marker marker = createMarker();
  g_marker_pub.publish(marker);

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

  // Publish an updated marker
  visualization_msgs::Marker marker = createMarker();
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
  bool dirty;

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
  if (d0 < TOLERANCE) { // Arrived at this goal
    dirty = true;
    g_goal_statuses[g_goals_index] = 1;
    ROS_INFO("FINISHED WAYPOINT %d: (%0.3f,%0.3f)", g_goals_index, g0.position.x, g0.position.y);
    g_goals_index++;

    // If we have more to do move forward
    if (g_goals_index < g_goals_length) {
      // Update to next goal
      g0 = g_goals[g_goals_index];
      ROS_INFO("START WAYPOINT %d: (%0.3f,%0.3f)", g_goals_index, g0.position.x, g0.position.y);

      // Recalculate math
      v0.x = g0.position.x - msg.pose.position.x;
      v0.y = g0.position.y - msg.pose.position.y;
      v0.z = g0.position.z - msg.pose.position.z;
      d0= std::sqrt(std::pow(v0.x, 2.0) + std::pow(v0.y, 2.0) + std::pow(v0.z, 2.0));
    } else { // Finished
      ROS_INFO("FINISHED ALL WAYPOINTS");
    }
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
      dirty = true;
      g_goal_statuses[g_goals_index] = 2;
      ROS_INFO("FINISHED WAYPOINT %d (PROMOTION): (%0.3f,%0.3f)", g_goals_index, g1.position.x, g1.position.y);
      g_goals_index++;
      ROS_INFO("START WAYPOINT %d: (%0.3f,%0.3f)", g_goals_index, g1.position.x, g1.position.y);
    }
  }

  if (dirty) {
    // Publish an updated marker
    visualization_msgs::Marker marker = createMarker();
    g_marker_pub.publish(marker);
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
