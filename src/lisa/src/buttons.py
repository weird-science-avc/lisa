#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Bool 

def buttons():
    pub = rospy.Publisher('button_1', Bool, queue_size=10)
    rospy.init_node('buttons', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % 1
        rospy.loginfo(hello_str)
        pub.publish(1)
        rate.sleep()

if __name__ == '__main__':
    try:
        buttons()
    except rospy.ROSInterruptException:
        pass
