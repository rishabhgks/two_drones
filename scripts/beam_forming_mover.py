#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float64

def talker():
    pub = rospy.Publisher('beam_forming_threshold', Float64, queue_size=10)
    rospy.init_node('beam_forming_node', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    start = rospy.get_rostime()
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % start.secs
        rospy.loginfo(hello_str)
        threshold = 0.4 if (rospy.get_rostime().secs - start.secs) < 20 else 0.8
        pub.publish(threshold)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass