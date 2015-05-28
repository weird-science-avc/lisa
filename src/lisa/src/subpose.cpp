// This program subscribes to turtle1/pose and shows its
// messages on the screen.
#include <string>
#include <sstream>
#include <iostream>

#include <curlpp/cURLpp.hpp>
#include <curlpp/Easy.hpp>
#include <curlpp/Options.hpp>

#include <ros/ros.h>
#include <weird_science/ackermann.h>
#include <iomanip> // for std::setprecision and std::fixed

// A callback function.  Executed each time a new pose
// message arrives.
void poseMessageReceived(const weird_science::ackermann& msg) {
  curlpp::Cleanup myCleanup;
  curlpp::Easy request;
  std::string url = "http:/1.2.3.4:1234/steering_angle/" +  msg.steering_angle;
  request.setOpt(new curlpp::options::Url(url));
  request.setOpt(new curlpp::options::PostFields("abcd"));
  request.setOpt(new curlpp::options::PostFieldSize(5));

  request.perform();
//  std::ostringstream os;
//  os << curlpp::options::Url(std::string("http://www.wikipedia.org"));
//  std::string asAskedInQuestion = os.str();
//  ROS_INFO_STREAM(asAskedInQuestion);

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


