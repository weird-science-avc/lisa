#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import UInt64
import Adafruit_BBIO.GPIO as GPIO

# This has to be push (in other words, when we publish this, the localizer should consume the node and run its calculations then).  The localizer can't calculate where we are until we push this info.  If we pull, we aren't guaranteed to have the most recent information published yet.

tick_count = 0
#TODO move this to configuration
encoder_pin = "P8_14"

def publish_tick(pub):
    global tick_count
    tick_count += 1
    pub.publish(tick_count)

def wheel_encoder():
    global encoder_pin
    pub = rospy.Publisher("lisa/sensors/wheel_encoder", UInt64, queue_size=10)
    #todo, register an interrupt here to get called when the hardware registers a wheel encoder
    #ros::Subscriber speed_sub = n.subscribe("lisa/cmd_speed", 1, speedCmdCallback);
    rospy.loginfo("Initializing Wheel encoder")
    rospy.init_node("wheel_encoder")
    #rate = rospy.Rate(10) # 10hz
    GPIO.setup(encoder_pin, GPIO.IN)
    while not rospy.is_shutdown():
        #wait_for_edge is blocking.  I'm not sure how ros handles that.
        #This also has speed issues.  We could get an edge while publish_tick is processing. It might make sense to hack into the library and see if we can get lower level or switch to c++ if it has better support.
        #This also is going to kill ros shutdown
        rospy.logdebug("Waiting for edge")
        GPIO.wait_for_edge(encoder_pin, GPIO.BOTH)
        rospy.logdebug("Rising edge detected")
        publish_tick(pub)
        #rate.sleep()

if __name__ == '__main__':
    try:
        wheel_encoder()
    except rospy.ROSInterruptException:
        GPIO.cleanup()
        pass
