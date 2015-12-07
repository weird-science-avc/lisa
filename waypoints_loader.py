#!/usr/bin/env python

import pprint
import rospy
import time

import geometry_msgs.msg
import std_srvs.srv

pp = pprint.PrettyPrinter(indent=2)

# Init right away
rospy.init_node('waypoint_loader', anonymous=True)

# TODO: Support loading from a file
waypoints = [
    # Right 10m square
    dict(x=8, y=0, tolerance=0.3),
    dict(x=10, y=-2, tolerance=0.3),
    dict(x=10, y=-8, tolerance=0.3),
    dict(x=8, y=-10, tolerance=0.3),
    dict(x=2, y=-10, tolerance=0.3),
    dict(x=0, y=-8, tolerance=0.3),
    dict(x=0, y=3, tolerance=0.3),
]

pp.pprint('hi world')
pp.pprint(waypoints)

# Initialize publishers and service proxies
initial_pose_pub = rospy.Publisher('/initialpose', geometry_msgs.msg.PoseWithCovarianceStamped, queue_size=1)
goal_pub = rospy.Publisher('/move_base_simple/goal', geometry_msgs.msg.PoseStamped, queue_size=10)
waypoints_clear = rospy.ServiceProxy('/waypoint_manager/clear', std_srvs.srv.Empty)
navigator_start = rospy.ServiceProxy('/navigator/start', std_srvs.srv.Empty)

# FIXME: Why is this necessary? I see some places in cpp too where publishing
# something like a visualization marker clear isn't seen either.
time.sleep(1)

# Clear any existing waypoints
waypoints_clear()

# Set our initial pose at origin
msg = geometry_msgs.msg.PoseWithCovarianceStamped()
msg.pose.pose.position.x = 0.0
msg.pose.pose.position.y = 0.0
msg.pose.pose.orientation.w = 1.0
initial_pose_pub.publish(msg)

# Publish the waypoints
for waypoint in waypoints:
    msg = geometry_msgs.msg.PoseStamped()
    msg.pose.position.x = waypoint['x']
    msg.pose.position.y = waypoint['y']
    goal_pub.publish(msg)

# Start the navigation
navigator_start()

pass
