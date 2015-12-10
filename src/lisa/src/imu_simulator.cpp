#include <angles/angles.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <tf/transform_datatypes.h>

#define PI 3.14159265
#define WHEEL_ENCODER_M_DISTANCE_FROM_TICKS 0.0544737

// Define max turning radius of car (66" right and 77" left, pick 80")
#define INCH_TO_M 0.0254
const double kMaxTurnRadius = 80.0 * INCH_TO_M;

double g_steering = 0.0; // [-1.0, 1.0] - unitless
double g_velocity = 0.0; // m/s
double g_yaw = 0.0;

void steeringCmdCallback(const std_msgs::Float64& msg) {
  g_steering = msg.data;
}

void velocityCmdCallback(const std_msgs::Float64& msg) {
  g_velocity = msg.data;
}

void initialposeCallback(const geometry_msgs::PoseWithCovarianceStamped& msg) {
  g_yaw = tf::getYaw(msg.pose.pose.orientation);
  ROS_INFO("[IMU SIMULATOR] yaw(initialpose)=%0.3f", g_yaw);
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "imu_simulator");
  ROS_INFO("imu_simulator started");
  ros::NodeHandle n;

  ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("lisa/sensors/imu", 1);
  ros::Subscriber steering_sub = n.subscribe("lisa/cmd_steering", 1, steeringCmdCallback);
  ros::Subscriber velocity_sub = n.subscribe("lisa/cmd_velocity", 1, velocityCmdCallback);

  // Subscribe to /initialpose so we can force the robot's orientation
  ros::Subscriber initalpose_sub = n.subscribe("initialpose", 1, initialposeCallback);

  // Initialize time stamp
  ros::Time stamp = ros::Time::now();

  // Loop to publish IMU updates as they should occur
  ros::Rate loop_rate(10); // Hz
  while (ros::ok())
  {
    ros::Time new_stamp = ros::Time::now();
    double dt = (new_stamp - stamp).toSec();
    stamp = new_stamp;

    // Only publish if we're steering and moving
    // TODO: Consider allowing no turn close to 0
    if (g_steering != 0.0 && g_velocity > 0.0) {
      // Find turn radius as inverse of steering
      double r = kMaxTurnRadius / g_steering;

      // w = v / r
      // wt = vt / r
      g_yaw = g_yaw + ((g_velocity * dt) / r);
      // TODO: Consider putting in angular and linear velocity
      sensor_msgs::Imu msg;
      msg.orientation = tf::createQuaternionMsgFromYaw(g_yaw);
      imu_pub.publish(msg);
      ROS_INFO("[IMU SIMULATOR] yaw=%0.3f", g_yaw);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
