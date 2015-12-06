#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/UInt64.h>

#define WHEEL_ENCODER_M_DISTANCE_FROM_TICKS 0.0544737

geometry_msgs::Twist twist;

void twistCallback(const geometry_msgs::Twist& msg) {
  twist = msg;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "wheel_encoder_simulator");
  ros::NodeHandle n;

  ros::Publisher wheel_encoder_pub = n.advertise<std_msgs::UInt64>("lisa/sensors/wheel_encoder", 1);
  ros::Subscriber twist_sub = n.subscribe("lisa/twist", 1, twistCallback);

  // Loop variables to accumulate distance and yaw
  double distance = 0.0;
  unsigned int last_ticks = 0;
  ros::Time stamp = ros::Time::now();

  ros::Rate loop_rate(1); // Hz
  while (ros::ok())
  {
    ros::Time new_stamp = ros::Time::now();
    double dt = (new_stamp - stamp).toSec();
    stamp = new_stamp;

    // Update exact distance
    double linear = twist.linear.x;
    distance += linear * dt;

    // Calculate new IMU ticks (rounded)
    unsigned int ticks = int(distance / WHEEL_ENCODER_M_DISTANCE_FROM_TICKS);
    ROS_DEBUG("dt=%0.3f, distance=%0.3f, ticks=%d", dt, distance, ticks);

    // Report if IMU ticks changed
    if (ticks > last_ticks) {
      ROS_INFO("[WHEEL ENCODER SIMULATOR] ticks=%d", ticks);
      std_msgs::UInt64 msg;
      msg.data = ticks;
      wheel_encoder_pub.publish(msg);

      // Update loop variable
      last_ticks = ticks;
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
