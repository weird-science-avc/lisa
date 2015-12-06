#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/UInt64.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>

// TODO: Doesn't something define this for us?
#define PI 3.14159265
#define WHEEL_ENCODER_M_DISTANCE_FROM_TICKS 0.0544737

// Global vars for storing wheel encoder and yaw
float g_yaw = 0.0;
unsigned int g_wheel_encoder_ticks = 0;

// Global pose
geometry_msgs::PoseStamped g_pose;

void wheelEncoderCallback(const std_msgs::UInt64& msg) {
  g_wheel_encoder_ticks = msg.data;
}

// quaternion to yaw
// (x,y,z,w) = (0, 0, sin(theta/2), cos(theta/2)).
void imuCallback(const sensor_msgs::Imu& msg) {
  g_yaw = tf::getYaw(msg.orientation);
}

void initialposeCallback(const geometry_msgs::PoseWithCovarianceStamped& msg) {
  g_pose.header.seq++;
  g_pose.header.stamp = msg.header.stamp;
  g_pose.pose = msg.pose.pose;
  g_yaw = tf::getYaw(g_pose.pose.orientation);
  ROS_INFO("POSITION(initial): (%.3f,%.3f):%.3f", g_pose.pose.position.x, g_pose.pose.position.y, g_yaw);
}

// TODO: Make a Localizer class to track state, see teleop_key.cpp for example.
int main(int argc, char **argv)
{
  ros::init(argc, argv, "localizer");
  ros::NodeHandle n;
  ros::Publisher pose_pub = n.advertise<geometry_msgs::PoseStamped>("lisa/pose", 1000);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("lisa/path", 10);
  tf::TransformBroadcaster pose_broadcaster;

  // Subscribe to sensor updates
  ros::Subscriber wheel_encoder_sub = n.subscribe("lisa/sensors/wheel_encoder", 1, wheelEncoderCallback);
  ros::Subscriber imu_sub = n.subscribe("lisa/sensors/imu", 1, imuCallback);

  // Subscribe to /initialpose so we can force the robot's position
  ros::Subscriber initalpose_sub = n.subscribe("initialpose", 1, initialposeCallback);

  // Initialize g_pose and publish
  g_pose.header.frame_id = "lisa";
  g_pose.header.seq = 0;
  g_pose.header.stamp = ros::Time::now();
  g_pose.pose.position.x = 0.0;
  g_pose.pose.position.y = 0.0;
  g_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
  pose_pub.publish(g_pose);
  ROS_DEBUG("POSITION(start): (0.000,0.000):0.000");

  // Track loop states
  float last_yaw = 0.0;
  int last_wheel_encoder_ticks = 0;

  // Setup path tracker
  visualization_msgs::Marker path_strip;
  path_strip.header.frame_id = "lisa";
  path_strip.ns = "path";
  path_strip.id = 0;
  path_strip.type = visualization_msgs::Marker::LINE_STRIP;
  path_strip.action = visualization_msgs::Marker::ADD;
  path_strip.pose = g_pose.pose;
  path_strip.scale.x = 0.1;
  path_strip.scale.y = 0.1;
  path_strip.scale.z = 0.1;
  path_strip.color.a = 1.0;
  path_strip.color.b = 1.0;

  ros::Rate loop_rate(10); // Hz
  while (ros::ok())
  {
    ros::Time now = ros::Time::now();

    // Figure out wheel encoder delta to determine if we need to update position
    unsigned int wheel_encoder_ticks = g_wheel_encoder_ticks;
    unsigned int wheel_encoder_delta = wheel_encoder_ticks - last_wheel_encoder_ticks;

    // Only update if moved
    if (wheel_encoder_delta > 0) {
      // Compute distance and angle change in radians
      float distance = WHEEL_ENCODER_M_DISTANCE_FROM_TICKS * float(wheel_encoder_delta);
      float yaw_delta = std::fmod(g_yaw - last_yaw, 2*PI);

      // Figure out position deltas based on straight or banked travel
      // TODO: Papers seem to indicate we can always do the straight estimate and call it good
      // TODO: Alternatively look at sensor simulators and try that math?
      float x_delta = 0.0;
      float y_delta = 0.0;
      if (yaw_delta == 0.0) {
        //ROS_DEBUG("Straight motion");
        x_delta = distance * cos(last_yaw);
        y_delta = distance * sin(last_yaw);
      } else {
        //ROS_DEBUG("Banked motion");
        // Get the turn radius from the distance and angle change
        float turnRadius = distance / yaw_delta;
        // Calculate x and y deltas as if we were at (0,0) pointed along x-axis
        // NOTE: x is 'up' for us, so that's governed by Sin
        float x_delta_origin = turnRadius * sin(yaw_delta);
        float y_delta_origin = turnRadius * (-cos(yaw_delta) + 1.0);

        // Now we need to rotate those deltas around (0,0) by our current orientation so they're correct
        float sin_r = sin(last_yaw);
        float cos_r = cos(last_yaw);
        x_delta = x_delta_origin * cos_r - y_delta_origin * sin_r;
        y_delta = x_delta_origin * sin_r + y_delta_origin * cos_r;
      }

      // Setup the transform to broadcast
      // TODO: Consider moving into a static transform broadcaster node
      geometry_msgs::TransformStamped pose_transform;
      pose_transform.header.stamp = now;
      pose_transform.header.frame_id = "lisa";
      pose_transform.child_frame_id = "map";
      pose_transform.transform.translation.x = 0.0;
      pose_transform.transform.translation.y = 0.0;
      pose_transform.transform.translation.z = 0.0;
      pose_transform.transform.rotation = tf::createQuaternionMsgFromYaw(0.0);
      pose_broadcaster.sendTransform(pose_transform);

      // Update g_pose and publish
      g_pose.header.stamp = now;
      g_pose.header.frame_id = "lisa";
      g_pose.header.seq++;
      g_pose.pose.position.x = g_pose.pose.position.x + x_delta;
      g_pose.pose.position.y = g_pose.pose.position.y + y_delta;
      g_pose.pose.orientation = tf::createQuaternionMsgFromYaw(g_yaw);
      ROS_DEBUG("QUATERNION: (%0.3f,%0.3f,%0.3f,%0.3f)",
          g_pose.pose.orientation.x,
          g_pose.pose.orientation.y,
          g_pose.pose.orientation.z,
          g_pose.pose.orientation.w
          );
      pose_pub.publish(g_pose);
      ROS_DEBUG("POSITION: (%.3f,%.3f):%.3f",
          g_pose.pose.position.x,
          g_pose.pose.position.y,
          g_yaw);

      // Update the path and publish
      path_strip.header.stamp = now;
      path_strip.points.push_back(g_pose.pose.position);
      marker_pub.publish(path_strip);

      // Save loop variables
      last_wheel_encoder_ticks = wheel_encoder_ticks;
      last_yaw = g_yaw;
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
