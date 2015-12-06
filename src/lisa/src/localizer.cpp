#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_datatypes.h"
#include "std_srvs/Empty.h"

#include <lisa/OdometrySensor.h>

// TODO: Doesn't something define this for us?
#define PI 3.14159265
#define WHEEL_ENCODER_M_DISTANCE_FROM_TICKS 0.0544737

// Global vars for storing wheel encoder and yaw
float gYaw = 0.0;
unsigned int gWheelEncoderTicks = 0;
geometry_msgs::PoseStamped gPose;

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
  gPose.pose.position.x = 0;
  gPose.pose.position.y = 0;
  gPose.pose.position.z = 0;
  // (x,y,z,w) = (0, 0, sin(theta/2), cos(theta/2)).
  gPose.pose.orientation = tf::createQuaternionMsgFromYaw(0);
  float yaw = tf::getYaw(gPose.pose.orientation);
  ROS_INFO("POSITION(RESET): (%.3f,%.3f):%.3f", gPose.pose.position.x, gPose.pose.position.y, yaw);
  return true;
}

void odometrySensorCallback(const lisa::OdometrySensor& sensor) {
  // Save wheel encoder and yaw
  gWheelEncoderTicks = sensor.wheel_encoder;
  gYaw = sensor.yaw;
}

// TODO: Make a Localizer class to track state, see teleop_key.cpp for example.
int main(int argc, char **argv)
{
  ros::init(argc, argv, "localizer");
  ros::NodeHandle n;
  ros::Publisher pose_pub = n.advertise<geometry_msgs::PoseStamped>("lisa/pose", 1000);
  // TODO: Since we only use on our loop calculations, we probably could just read from a service;
  //  or perhaps each callback we should recalculate position and then publish at frequency.
  ros::Subscriber sensor_sub = n.subscribe("lisa/sensors/odometry", 1, odometrySensorCallback);
  ros::ServiceServer service = n.advertiseService("lisa/localizer/reset", reset);
  ros::Rate loop_rate(10); // Hz

  // Track loop states
  float lastYaw = 0.0;
  int lastWheelEncoderTicks = 0;
  geometry_msgs::PoseStamped lastPublishedPose;
  lastPublishedPose.header.stamp = ros::Time::now();
  lastPublishedPose.header.frame_id = "lisa";
  pose_pub.publish(lastPublishedPose);
  while (ros::ok())
  {
    // Figure out IMU's latest orientation, figure out thetaDelta, and updated stored value
    // NOTE: IMU has right positive, so do last - now instead of more normal now - last to make left positive again.
    float yaw = gYaw;
    float thetaDelta = normalizeRadians((lastYaw - yaw) * PI / 180.0);

    // Figure out wheel encoder delta, update stored value and calculate distance
    unsigned int wheelEncoderTicks = gWheelEncoderTicks;
    unsigned int wheelEncoderDelta = wheelEncoderTicks - lastWheelEncoderTicks;
    float distance = WHEEL_ENCODER_M_DISTANCE_FROM_TICKS * float(wheelEncoderDelta);

    // Figure out position deltas based on straight or banked travel
    // TODO: Papers seem to indicate we can always do the straight estimate and call it good
    float xDelta = 0.0;
    float yDelta = 0.0;
    if (thetaDelta == 0.0) {
      //ROS_DEBUG("Straight motion");
      xDelta = distance * cos(lastYaw);
      yDelta = distance * sin(lastYaw);
    } else {
      //ROS_DEBUG("Banked motion");
      // Get the turn radius from the distance and angle change
      float turnRadius = distance / thetaDelta;
      // Calculate x and y deltas as if we were at (0,0) pointed along x-axis
      // NOTE: x is 'up' for us, so that's governed by Sin
      float xDeltaOrigin = turnRadius * sin(thetaDelta);
      float yDeltaOrigin = turnRadius * (-cos(thetaDelta) + 1.0);

      // Now we need to rotate those deltas around (0,0) by our current orientation so they're correct
      float sinR = sin(lastYaw);
      float cosR = cos(lastYaw);
      xDelta = xDeltaOrigin * cosR - yDeltaOrigin * sinR;
      yDelta = xDeltaOrigin * sinR + yDeltaOrigin * cosR;
    }

    // Update gPose
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = gPose.pose.position.x + xDelta;
    pose.pose.position.y = gPose.pose.position.y + yDelta;
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

    // Save loop variables
    lastWheelEncoderTicks = wheelEncoderTicks;
    lastYaw = yaw;

    // Only publish changes
    if (!poseEql(pose, lastPublishedPose)) {
      pose.header.seq = lastPublishedPose.header.seq + 1;
      pose.header.stamp = ros::Time::now();
      pose.header.frame_id = "lisa";
      pose_pub.publish(pose);
      ROS_DEBUG("POSITION: (%.3f,%.3f):%.3f", pose.pose.position.x, pose.pose.position.y, yaw);
      lastPublishedPose = pose;
    }
    gPose = pose;

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
