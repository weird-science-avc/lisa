#!/usr/bin/env python
# license removed for brevity
import rospy
import serial
#from sensor_msgs import Imu
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

def publish_orientation(v):
    global orientation
    rospy.loginfo(v)
    # pub.publish(tick_count)

def imu():
    # pub = rospy.Publisher("lisa/sensors/imu", Imu, queue_size=10)
    times_per_second = 10
    rospy.loginfo("Initializing IMU")
    rospy.init_node("imu")
    rate = rospy.Rate(times_per_second) # 10hz
    UART.setup("UART1")
     
    ser = serial.Serial(port = "/dev/ttyO1", baudrate=115200 )
    ser.close()
    ser.open()
    if ser.isOpen():
    	print "Serial is open!"
    
    while not rospy.is_shutdown():
	    ser.flushInput()
        str = ser.read(116)
        imu_output = str.split('\n')[-2]
        publish_orientation(imu_output)
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



# void serialEvent() {
#   if (!MOCK_IMU) {
#     float roll = Serial.parseFloat();
#     float pitch = Serial.parseFloat();
#     float yaw = Serial.parseFloat();
#     char garbage[20];
#     Serial.readBytesUntil('\0', garbage, 20);
#     //Serial.print("gYaw: ");
#     //Serial.println(gYaw);
#     // NOTE: Sometimes the IMU spikes changes so limit the size we believe for a given fast loop rad
#     //if (abs(yaw - gYaw) < IMU_MAX_DELTA_DEGREES) {
#       gYaw = yaw;
#     //}
#   }
# }
