#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/UInt64.h>
#include <std_srvs/Empty.h>
#include <tf/transform_datatypes.h>

// TODO: Doesn't something define this for us?
#define PI 3.14159265
#define WHEEL_ENCODER_M_DISTANCE_FROM_TICKS 0.0544737

// Global vars for storing wheel encoder and yaw
float g_yaw = 0.0;
unsigned int g_wheel_encoder_ticks = 0;
geometry_msgs::PoseStamped g_pose;

// TODO: Overload equality operator
bool poseEql(geometry_msgs::PoseStamped p1, geometry_msgs::PoseStamped p2) {
  return p1.pose.position.x == p2.pose.position.x &&
    p1.pose.position.y == p2.pose.position.y &&
    p1.pose.position.z == p2.pose.position.z &&
    p1.pose.orientation.x == p2.pose.orientation.x &&
    p1.pose.orientation.y == p2.pose.orientation.y &&
    p1.pose.orientation.z == p2.pose.orientation.z &&
    p1.pose.orientation.w == p2.pose.orientation.w;
}

// Move to utility functions
float normalizeRadians(float r) {
  while (r > 2 * PI) { r -= 2 * PI; }
  while (r < -2 * PI) { r += 2 * PI; }
  return r;
}

bool reset(std_srvs::Empty::Request  &req,
           std_srvs::Empty::Response &res) {
  g_pose.pose.position.x = 0;
  g_pose.pose.position.y = 0;
  g_pose.pose.position.z = 0;
  // (x,y,z,w) = (0, 0, sin(theta/2), cos(theta/2)).
  g_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);
  float yaw = tf::getYaw(g_pose.pose.orientation);
  ROS_INFO("POSITION(RESET): (%.3f,%.3f):%.3f", g_pose.pose.position.x, g_pose.pose.position.y, yaw);
  return true;
}

void wheelEncoderCallback(const std_msgs::UInt64& msg) {
  g_wheel_encoder_ticks = msg.data;
}

void imuCallback(const sensor_msgs::Imu& msg) {
  g_yaw = tf::getYaw(msg.orientation);
}

// TODO: Make a Localizer class to track state, see teleop_key.cpp for example.
int main(int argc, char **argv)
{
  ros::init(argc, argv, "localizer");
  ros::NodeHandle n;
  ros::Publisher pose_pub = n.advertise<geometry_msgs::PoseStamped>("lisa/pose", 1000);
  ros::Subscriber wheel_encoder_sub = n.subscribe("lisa/sensors/wheel_encoder", 1, wheelEncoderCallback);
  ros::Subscriber imu_sub = n.subscribe("lisa/sensors/imu", 1, imuCallback);
  ros::ServiceServer service = n.advertiseService("lisa/localizer/reset", reset);
  ros::Rate loop_rate(10); // Hz

  // Track loop states
  float last_yaw = 0.0;
  int last_wheel_encoder_ticks = 0;
  geometry_msgs::PoseStamped last_published_pose;
  last_published_pose.header.stamp = ros::Time::now();
  last_published_pose.header.frame_id = "lisa";
  pose_pub.publish(last_published_pose);
  while (ros::ok())
  {
    // Figure out IMU's latest orientation, figure out theta_delta, and updated stored value
    // NOTE: IMU has right positive, so do last - now instead of more normal now - last to make left positive again.
    float yaw = g_yaw;
    float theta_delta = normalizeRadians((last_yaw - yaw) * PI / 180.0);

    // Figure out wheel encoder delta, update stored value and calculate distance
    unsigned int wheel_encoder_ticks = g_wheel_encoder_ticks;
    unsigned int wheel_encoder_delta = wheel_encoder_ticks - last_wheel_encoder_ticks;
    float distance = WHEEL_ENCODER_M_DISTANCE_FROM_TICKS * float(wheel_encoder_delta);

    // Figure out position deltas based on straight or banked travel
    // TODO: Papers seem to indicate we can always do the straight estimate and call it good
    float x_delta = 0.0;
    float y_delta = 0.0;
    if (theta_delta == 0.0) {
      //ROS_DEBUG("Straight motion");
      x_delta = distance * cos(last_yaw);
      y_delta = distance * sin(last_yaw);
    } else {
      //ROS_DEBUG("Banked motion");
      // Get the turn radius from the distance and angle change
      float turnRadius = distance / theta_delta;
      // Calculate x and y deltas as if we were at (0,0) pointed along x-axis
      // NOTE: x is 'up' for us, so that's governed by Sin
      float x_delta_origin = turnRadius * sin(theta_delta);
      float y_delta_origin = turnRadius * (-cos(theta_delta) + 1.0);

      // Now we need to rotate those deltas around (0,0) by our current orientation so they're correct
      float sin_r = sin(last_yaw);
      float cos_r = cos(last_yaw);
      x_delta = x_delta_origin * cos_r - y_delta_origin * sin_r;
      y_delta = x_delta_origin * sin_r + y_delta_origin * cos_r;
    }

    // Update g_pose
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = g_pose.pose.position.x + x_delta;
    pose.pose.position.y = g_pose.pose.position.y + y_delta;
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

    // Save loop variables
    last_wheel_encoder_ticks = wheel_encoder_ticks;
    last_yaw = yaw;

    // Only publish changes
    if (!poseEql(pose, last_published_pose)) {
      pose.header.seq = last_published_pose.header.seq + 1;
      pose.header.stamp = ros::Time::now();
      pose.header.frame_id = "lisa";
      pose_pub.publish(pose);
      ROS_DEBUG("POSITION: (%.3f,%.3f):%.3f", pose.pose.position.x, pose.pose.position.y, yaw);
      last_published_pose = pose;
    }
    g_pose = pose;

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
