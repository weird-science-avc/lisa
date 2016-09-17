#!/usr/bin/env python
# license removed for brevity
import rospy, tf
import serial
import re
import math
from sensor_msgs.msg import Imu
import Adafruit_BBIO.GPIO as GPIO
import Adafruit_BBIO.UART as UART


# msg = Imu()
#
#       sensor_msgs::Imu msg;
#       msg.orientation = tf::createQuaternionMsgFromYaw(g_yaw);
#       imu_pub.publish(msg);
#       ROS_INFO("[IMU SIMULATOR] yaw=%0.3f", g_yaw);

# This has to be push (in other words, when we publish this, the localizer should consume the node and run its calculations then).  The localizer can't calculate where we are until we push this info.  If we pull, we aren't guaranteed to have the most recent information published yet.

orientation = ""  #This will need changed to the actual message

def publish_orientation(pub, v):
    global orientation
    pub.publish(v)

def imu():
    pub = rospy.Publisher("lisa/sensors/imu", Imu, queue_size=10)
    times_per_second = 5
    bytes_to_read = 130  #This is slightly over twice the length of a single line of data.  A single line is ~58 characters.

    rospy.loginfo("Initializing IMU")
    rospy.init_node("imu")
    rate = rospy.Rate(times_per_second) # 10hz
    UART.setup("UART1")
    ser = serial.Serial(port = "/dev/ttyO1", baudrate=115200)
    ser.close()
    ser.open()
    if ser.isOpen():
    	print "Serial is open!"

    while not rospy.is_shutdown():
        ser.flushInput() #The imu blasts us with data and fills up the serial input buffer.  We clear it out here to wait for new values
        stream = ser.read(bytes_to_read)
        full_orientation = stream.split('\n')[-2] #Since we're getting a stream of data, we don't know whether we're starting at the beginning of the line or the middle. This throws out the last partial line and gets the previous full line
        yaw_deg = float(re.split('\s*', full_orientation)[-2])
        rospy.loginfo("yaw=%0.3f", yaw_deg)
        # correct backwards yaw
        #yaw = yaw_deg * math.pi / 180.0
        yaw = -1.0 * yaw_deg * math.pi / 180.0
        q = tf.transformations.quaternion_from_euler(0, 0, yaw)
        o = Imu()
        o.orientation.x = q[0]
        o.orientation.y = q[1]
        o.orientation.z = q[2]
        o.orientation.w = q[3]
        publish_orientation(pub, o)
        rate.sleep()

    ser.close()

if __name__ == '__main__':
    try:
        imu()
    except rospy.ROSInterruptException:
        # Eventually, you'll want to clean up, but leave this commented for now,
        # as it doesn't work yet - maybe it does?
        #UART.cleanup()
        pass

