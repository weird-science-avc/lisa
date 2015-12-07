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

// Global pose and path
ros::Publisher g_pose_pub;
ros::Publisher g_marker_pub;
geometry_msgs::PoseStamped g_pose;
visualization_msgs::Marker g_path;
int g_pose_index = 0;

void wheelEncoderCallback(const std_msgs::UInt64& msg) {
  g_wheel_encoder_ticks = msg.data;
}

// quaternion to yaw
// (x,y,z,w) = (0, 0, sin(theta/2), cos(theta/2)).
void imuCallback(const sensor_msgs::Imu& msg) {
  g_yaw = tf::getYaw(msg.orientation);
}

void initialposeCallback(const geometry_msgs::PoseWithCovarianceStamped& msg) {
  //g_pose_index++;

  // Set pose and path
  g_pose.header.seq = g_pose_index;
  g_pose.pose = msg.pose.pose;
  g_pose_pub.publish(g_pose);

  g_path.header.seq = g_pose_index;
  g_path.points.clear();
  g_marker_pub.publish(g_path);

  g_yaw = tf::getYaw(g_pose.pose.orientation);
  ROS_INFO("POSITION(initial): (%.3f,%.3f):%.3f", g_pose.pose.position.x, g_pose.pose.position.y, g_yaw);
}

// TODO: Make a Localizer class to track state, see teleop_key.cpp for example.
int main(int argc, char **argv) {
  ros::init(argc, argv, "localizer");
  ROS_INFO("localizer started");
  ros::NodeHandle n;

  // Setup publishers:
  g_pose_pub = n.advertise<geometry_msgs::PoseStamped>("lisa/pose", 1000);
  g_marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 0);

  // Setup subscribers:
  // - /lisa/sensors/wheel_encoder - Wheel Encoder data
  ros::Subscriber wheel_encoder_sub = n.subscribe("lisa/sensors/wheel_encoder", 1, wheelEncoderCallback);
  // - /lisa/sensors/imu - IMU data
  ros::Subscriber imu_sub = n.subscribe("lisa/sensors/imu", 1, imuCallback);
  // - /initialpose - so we can force the robot's position
  ros::Subscriber initalpose_sub = n.subscribe("initialpose", 1, initialposeCallback);

  // Initialize g_pose and publish
  g_pose.header.frame_id = "lisa";
  g_pose.header.seq = g_pose_index;
  g_pose.header.stamp = ros::Time();
  g_pose.pose.position.x = 0.0;
  g_pose.pose.position.y = 0.0;
  g_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
  g_pose_pub.publish(g_pose);
  ROS_DEBUG("POSITION(start): (0.000,0.000):0.000");

  // Initialize g_path and publish
  g_path.header.frame_id = "lisa";
  g_path.header.seq = g_pose_index;
  g_path.header.stamp = ros::Time();
  g_path.ns = "path";
  g_path.id = 0;
  g_path.type = visualization_msgs::Marker::LINE_STRIP;
  g_path.action = visualization_msgs::Marker::ADD;
  g_path.scale.x = 0.1;
  g_path.scale.y = 0.1;
  g_path.scale.z = 0.1;
  g_path.color.a = 1.0;
  g_path.color.b = 1.0;
  g_path.points.push_back(g_pose.pose.position);
  g_marker_pub.publish(g_path);

  // Track loop states
  float last_yaw = 0.0;
  int last_wheel_encoder_ticks = 0;

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

      // Update g_pose and publish
      g_pose.pose.position.x = g_pose.pose.position.x + x_delta;
      g_pose.pose.position.y = g_pose.pose.position.y + y_delta;
      g_pose.pose.orientation = tf::createQuaternionMsgFromYaw(g_yaw);
      ROS_DEBUG("QUATERNION: (%0.3f,%0.3f,%0.3f,%0.3f)",
          g_pose.pose.orientation.x,
          g_pose.pose.orientation.y,
          g_pose.pose.orientation.z,
          g_pose.pose.orientation.w
          );
      g_pose_pub.publish(g_pose);
      ROS_DEBUG("POSITION: (%.3f,%.3f):%.3f",
          g_pose.pose.position.x,
          g_pose.pose.position.y,
          g_yaw);

      // Update the path and publish
      g_path.points.push_back(g_pose.pose.position);
      g_marker_pub.publish(g_path);

      // Save loop variables
      last_wheel_encoder_ticks = wheel_encoder_ticks;
      last_yaw = g_yaw;
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
