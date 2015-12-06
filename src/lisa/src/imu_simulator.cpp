#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>

#define PI 3.14159265
#define WHEEL_ENCODER_M_DISTANCE_FROM_TICKS 0.0544737

double g_yaw = 0.0;
geometry_msgs::Twist g_twist;

void twistCallback(const geometry_msgs::Twist& msg) {
  g_twist = msg;
}

void initialposeCallback(const geometry_msgs::PoseWithCovarianceStamped& msg) {
  g_yaw = tf::getYaw(msg.pose.pose.orientation);
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "imu_simulator");
  ros::NodeHandle n;

  ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("lisa/sensors/imu", 1);
  ros::Subscriber twist_sub = n.subscribe("lisa/twist", 1, twistCallback);

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

    // Update yaw based on current twist if we have angular velocity
    double angular = g_twist.angular.x;
    ROS_DEBUG("[IMU SIMULATOR] angular=%0.3f (dt=%0.3fs)", angular, dt);
    if (angular != 0.0) {
      g_yaw = std::fmod(g_yaw + angular * dt, 2*PI);

      sensor_msgs::Imu msg;
      msg.orientation = tf::createQuaternionMsgFromYaw(g_yaw);
      // TODO: Consider putting in angular and linear velocity
      imu_pub.publish(msg);
      ROS_INFO("[IMU SIMULATOR] yaw=%0.3f", g_yaw);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
