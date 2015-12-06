#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>

#define PI 3.14159265
#define WHEEL_ENCODER_M_DISTANCE_FROM_TICKS 0.0544737

geometry_msgs::Twist twist;

void twistCallback(const geometry_msgs::Twist& msg) {
  twist = msg;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "imu_simulator");
  ros::NodeHandle n;

  ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("lisa/sensors/imu", 1);
  ros::Subscriber twist_sub = n.subscribe("lisa/twist", 1, twistCallback);

  // Loop variables to accumulate distance and yaw
  double distance, yaw = 0.0;
  double last_yaw = 0.0;
  ros::Time stamp = ros::Time::now();

  ros::Rate loop_rate(1); // Hz
  while (ros::ok())
  {
    ros::Time new_stamp = ros::Time::now();
    double dt = (new_stamp - stamp).toSec();
    stamp = new_stamp;

    // Update yaw based on current twist
    double linear = twist.linear.x;
    double angular = twist.angular.x;
    yaw = std::fmod(yaw + angular * dt, 2*PI);
    ROS_DEBUG("dt=%0.3f, yaw=%0.3f", dt, yaw);

    // Report if yaw changed
    if (yaw != last_yaw) {
      ROS_INFO("[IMU SIMULATOR] yaw=%0.3f", yaw);
      sensor_msgs::Imu msg;
      msg.orientation = tf::createQuaternionMsgFromYaw(yaw);
      // TODO: Consider putting in angular and linear velocity
      imu_pub.publish(msg);

      // Update loop variable
      last_yaw = yaw;
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
