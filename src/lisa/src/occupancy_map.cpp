#include <iostream>
#include <fstream>
#include <string>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose.h>
using namespace std;

int main(int argc, char** argv) {
  ifstream map_file;
  map_file.open ("/home/jon/Documents/lisa/src/lisa/data/sparkfun.map");
  string line;
  std::vector<signed char> map (3500);
  int arr[3500];
  int i = 0;
  while ( getline(map_file, line, ',') )
    {
      map[i] = atoi(line.c_str());
      i++;
    }
  map_file.close();

  ros::init(argc, argv, "occupancy_map");
  ROS_INFO("occupancy_map started");

  ros::NodeHandle n;
  nav_msgs::OccupancyGrid occupancy_map; 

  ros::Publisher occupancy_map_pub = n.advertise<nav_msgs::OccupancyGrid>("lisa/occupancy_map", 1);

  // Initialize time stamp
  ros::Time stamp = ros::Time::now();

  // Initialize occupancy_map_pub and publish
  unsigned int map_width_meters = 150;
  unsigned int map_height_meters = 200;
  occupancy_map.info.map_load_time = ros::Time::now();
  occupancy_map.info.resolution = 1; // meters per cell
  occupancy_map.info.width = 50;//map_width_meters / occupancy_map_pub.info.resolution ;
  occupancy_map.info.height = 70;//map_height_meters / occupancy_map_pub.info.resolution ;
  occupancy_map.info.origin.position.x = -25;
  occupancy_map.info.origin.position.y = -10;
  occupancy_map.info.origin.orientation.w = 90;
  occupancy_map.header.frame_id = "lisa";
  occupancy_map.header.seq = 1; // Do we really need this?
  occupancy_map.header.stamp = ros::Time();
  occupancy_map.data = map;
  occupancy_map_pub.publish(occupancy_map);
  ROS_DEBUG("Loaded initial map.");

  ros::Rate loop_rate(1); // Hz
  while (ros::ok()) // This loop shouldn't be necessary, but rviz would not receive the published message until I added this
  {
    occupancy_map_pub.publish(occupancy_map);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
