#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <lisa/OdometrySensor.h>

#define PI 3.14159265
#define WHEEL_ENCODER_M_DISTANCE_FROM_TICKS 0.0544737

geometry_msgs::Twist twist;

void twistCallback(const geometry_msgs::Twist& msg) {
  twist = msg;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "sensor_simulator");
  ros::NodeHandle n;

  ros::Publisher sensor_pub = n.advertise<lisa::OdometrySensor>("lisa/sensors/odometry", 1);
  ros::Subscriber twist_sub = n.subscribe("lisa/twist", 1, twistCallback);

  // Loop variables to accumulate distance and yaw
  double distance, yaw = 0.0;
  unsigned int last_ticks = 0;
  ros::Time stamp = ros::Time::now();

  ros::Rate loop_rate(1); // Hz
  while (ros::ok())
  {
    ros::Time new_stamp = ros::Time::now();
    double dt = (new_stamp - stamp).toSec();
    stamp = new_stamp;

    // Update distance and yaw based on current twist
    double linear = twist.linear.x;
    double angular = twist.angular.x;
    distance += linear * dt;
    yaw = std::fmod(yaw + angular * dt, 2*PI);

    // Calculate new IMU ticks
    unsigned int ticks = int(distance / WHEEL_ENCODER_M_DISTANCE_FROM_TICKS);
    ROS_DEBUG("dt=%0.3f, distance=%0.3f, yaw=%0.3f, ticks=%d", dt, distance, yaw, ticks);

    // Report if IMU ticks changed
    if (ticks > last_ticks) {
      lisa::OdometrySensor sensor;
      sensor.wheel_encoder = ticks;
      sensor.yaw = yaw;
      sensor_pub.publish(sensor);

      // Update loop variable
      last_ticks = ticks;
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
