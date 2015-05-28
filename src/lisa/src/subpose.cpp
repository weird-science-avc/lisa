// This program subscribes to turtle1/pose and shows its
// messages on the screen.
#include <ros/ros.h>
#include <weird_science/ackermann.h>
#include <iomanip> // for std::setprecision and std::fixed

// A callback function.  Executed each time a new pose
// message arrives.
void poseMessageReceived(const weird_science::ackermann& msg) {
  ROS_INFO_STREAM(std::setprecision(2) << std::fixed
    << "steering_angle=(" <<  msg.steering_angle <<  ")"
    << " speed=" << msg.speed);
}

int main(int argc, char **argv) {
  // Initialize the ROS system and become a node.
  ros::init(argc, argv, "subscribe_to_ackerman");
  ros::NodeHandle nh;

  // Create a subscriber object.
  ros::Subscriber sub = nh.subscribe("lisa1/ackermann", 1000,
    &poseMessageReceived);

  // Let ROS take over.
  ros::spin();
}


