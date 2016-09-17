#!/usr/bin/env python
# license removed for brevity
import json

import rospy
import geometry_msgs.msg
import std_srvs.srv

CLEAR_WAYPOINTS_RPC = "/waypoint_manager/clear"
RESTART_WAYPOINTS_RPC = "/waypoint_manager/restart"
START_NAV_RPC = "/navigator/start"
INITIAL_POSE_TOPIC = "initialpose"
POSE_TOPIC = "/move_base_simple/goal"
DEFAULT_COURSE_MAP_PARAMETER = "/hal/map_location"
DEFAULT_NODE_NAME = "hal9000"


def load_course(parameter):
    file_location = rospy.get_param(parameter, "/root/weird_science/waypoints.json")
    with open(file_location, 'r') as f:
        yolo = json.loads(f.read())
        return yolo.get("waypoints", [])


def publish_waypoint(topic, waypoint):
    msg = geometry_msgs.msg.PoseStamped()
    msg.pose.position.x = waypoint['x']
    msg.pose.position.y = waypoint['y']
    print("PUBLISHING: ", waypoint)
    topic.publish(msg)


def publish_course(topic, course):
    for waypoint in course:
        publish_waypoint(topic, waypoint)


def set_initial_position(topic):
    msg = geometry_msgs.msg.PoseWithCovarianceStamped()
    msg.pose.pose.position.x = 0.0
    msg.pose.pose.position.y = 0.0
    msg.pose.pose.orientation.w = 0.0
    topic.publish(msg)


def open_the_bay_doors():
    initial_position_topic = rospy.Publisher(INITIAL_POSE_TOPIC,
        geometry_msgs.msg.PoseWithCovarianceStamped, queue_size=10)
    position_topic = rospy.Publisher(POSE_TOPIC,
        geometry_msgs.msg.PoseStamped, queue_size=10)
    clear_waypoints = rospy.ServiceProxy(CLEAR_WAYPOINTS_RPC, std_srvs.srv.Empty)
    restart_waypoints = rospy.ServiceProxy(RESTART_WAYPOINTS_RPC, std_srvs.srv.Empty)
    start_navigation = rospy.ServiceProxy(START_NAV_RPC, std_srvs.srv.Empty)

    rospy.loginfo("starting /hal")
    rospy.init_node(DEFAULT_NODE_NAME)
    print("STARTED HAL NODE")
    # read waypoint file param
    course = load_course(DEFAULT_COURSE_MAP_PARAMETER)
    print("LOADED COURSE")
    clear_waypoints()
    print("CLEARED WAYPOINTS")
    publish_course(position_topic, course)
    print("PUBLISHING COURSE")
    restart_waypoints()
    print("RESTARTED WAYPOINTS")
    set_initial_position(initial_position_topic)
    print("SET INITIAL POSITION")
    print("STARTING NAVIGATION...")
    start_navigation()

if __name__ == '__main__':
    open_the_bay_doors()
