#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/UInt32.h"

// TODO: Doesn't something define this for us?
#define PI 3.14159265
#define WHEEL_ENCODER_M_DISTANCE_FROM_TICKS 0.0544737

// Global vars for storing wheel encoder and yaw
float gYaw = 0.0;
unsigned int gWheelEncoderTicks = 0;

// TODO: Overload equality operator
bool poseEql(geometry_msgs::Pose2D p1, geometry_msgs::Pose2D p2) {
  return p1.x != p2.x || p1.y != p2.y || p1.theta != p2.theta;
}

// Move to utility functions
float normalizeRadians(float r) {
  while (r > 2 * PI) { r -= 2 * PI; }
  while (r < -2 * PI) { r += 2 * PI; }
  return r;
}

void imuCallback(const sensor_msgs::Imu& msg) {
  // Convert quaternion orientation into the yaw we care about
  // TODO: Consider if we should use a 3D pose instead and just store quaternion
  gYaw = 0;
}

void wheelEncoderCallback(const std_msgs::UInt32& wheelEncoder) {
  gWheelEncoderTicks = wheelEncoder.data;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "localizer");
  ros::NodeHandle n;
  ros::Publisher pubPose = n.advertise<geometry_msgs::Pose2D>("lisa/pose", 1000);
  ros::Subscriber subImu = n.subscribe("lisa/sensors/imu", 1, imuCallback);
  ros::Subscriber subWheelEncoder = n.subscribe("lisa/sensors/wheel_encoder", 1, wheelEncoderCallback);
  ros::Rate loop_rate(10); // Hz

  // Track loop states
  // TODO: Perhaps have nodes that publish wheel encoder and IMU info that we subscribe to
  float lastYaw = 0.0;
  int lastWheelEncoderTicks = 0;
  geometry_msgs::Pose2D pose;
  while (ros::ok())
  {
    // Figure out IMU's latest orientation, figure out thetaDelta, and updated stored value
    // NOTE: IMU has right positive, so do last - now instead of more normal now - last to make left positive again.
    float yaw = gYaw;
    float thetaDelta = normalizeRadians((lastYaw - yaw) * PI / 180.0);
    lastYaw = yaw;

    // Figure out wheel encoder delta, update stored value and calculate distance
    unsigned int wheelEncoderTicks = gWheelEncoderTicks;
    unsigned int wheelEncoderDelta = wheelEncoderTicks - lastWheelEncoderTicks;
    lastWheelEncoderTicks = wheelEncoderTicks;
    float distance = WHEEL_ENCODER_M_DISTANCE_FROM_TICKS * float(wheelEncoderDelta);

    // Figure out position deltas based on straight or banked travel
    // TODO: Papers seem to indicate we can always do the straight estimate and call it good
    float xDelta = 0.0;
    float yDelta = 0.0;
    if (thetaDelta == 0.0) {
      //ROS_DEBUG("Straight motion");
      xDelta = distance * cos(pose.theta);
      yDelta = distance * sin(pose.theta);
    } else {
      //ROS_DEBUG("Banked motion");
      // Get the turn radius from the distance and angle change
      float turnRadius = distance / thetaDelta;
      // Calculate x and y deltas as if we were at (0,0) pointed along x-axis
      // NOTE: x is 'up' for us, so that's governed by Sin
      float xDeltaOrigin = turnRadius * sin(thetaDelta);
      float yDeltaOrigin = turnRadius * (-cos(thetaDelta) + 1.0);

      // Now we need to rotate those deltas around (0,0) by our current orientation so they're correct
      float sinR = sin(pose.theta);
      float cosR = cos(pose.theta);
      xDelta = xDeltaOrigin * cosR - yDeltaOrigin * sinR;
      yDelta = xDeltaOrigin * sinR + yDeltaOrigin * cosR;
    }

    // Update pose
    geometry_msgs::Pose2D newPose;
    newPose.x = pose.x + xDelta;
    newPose.y = pose.y + yDelta;
    newPose.theta = normalizeRadians(pose.theta + thetaDelta);
    // Only publish changes
    if (poseEql(newPose, pose)) {
      pubPose.publish(newPose);
      ROS_DEBUG("POSITION: (%.3f,%.3f):%.3f", newPose.x, newPose.y, newPose.theta);
    }
    pose = newPose;

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
