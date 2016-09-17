#!/usr/bin/env python
# license removed for brevity
import json
import time

import rospy
import geometry_msgs.msg
import std_srvs.srv
from Adafruit_BBIO import GPIO

CLEAR_WAYPOINTS_RPC = "/waypoint_manager/clear"
RESTART_WAYPOINTS_RPC = "/waypoint_manager/restart"
START_NAV_RPC = "/navigator/start"
STOP_NAV_RPC = "/navigator/stop"
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
    time.sleep(1)


def publish_course(topic, course):
    for waypoint in course:
        publish_waypoint(topic, waypoint)


def set_initial_position(topic):
    msg = geometry_msgs.msg.PoseWithCovarianceStamped()
    msg.pose.pose.position.x = 0.0
    msg.pose.pose.position.y = 0.0
    msg.pose.pose.orientation.w = 1.0
    topic.publish(msg)


def setup_hardware():
    GPIO.setup("P8_12", GPIO.IN)


def open_the_bay_doors():
    initial_position_topic = rospy.Publisher(INITIAL_POSE_TOPIC,
        geometry_msgs.msg.PoseWithCovarianceStamped, queue_size=10)
    position_topic = rospy.Publisher(POSE_TOPIC,
        geometry_msgs.msg.PoseStamped, queue_size=10)
    clear_waypoints = rospy.ServiceProxy(CLEAR_WAYPOINTS_RPC, std_srvs.srv.Empty)
    restart_waypoints = rospy.ServiceProxy(RESTART_WAYPOINTS_RPC, std_srvs.srv.Empty)
    start_navigation = rospy.ServiceProxy(START_NAV_RPC, std_srvs.srv.Empty)
    stop_navigation = rospy.ServiceProxy(STOP_NAV_RPC, std_srvs.srv.Empty)

    rospy.loginfo("starting /hal")
    rospy.init_node(DEFAULT_NODE_NAME)
    print("STARTED HAL NODE")

    setup_hardware()
    button_state = 0
    running_state = 0
    while not rospy.is_shutdown():
        new_state = GPIO.input("P8_12")
        if new_state == 1 and button_state == 0 and running_state == 0:
            running_state = 1
            print("LOADING COURSE")
            course = load_course(DEFAULT_COURSE_MAP_PARAMETER)
            print("CLEARING WAYPOINTS")
            clear_waypoints()
            print("SETTING INITIAL POSITION")
            set_initial_position(initial_position_topic)
            print("PUBLISHING COURSE")
            publish_course(position_topic, course)
            print("RESTARTING WAYPOINTS")
            restart_waypoints()
            print("STARTING NAVIGATION...")
            start_navigation()
        elif new_state == 1 and button_state == 0 and running_state == 1:
            running_state = 0
            stop_navigation()
        button_state = new_state

if __name__ == '__main__':
    open_the_bay_doors()
