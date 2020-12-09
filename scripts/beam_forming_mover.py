#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float32

def talker():
    pub = rospy.Publisher('beam_forming_threshold', Float32, queue_size=10)
    rospy.init_node('beam_forming_mover', anonymous=True)
    rate = rospy.Rate(100) # 10hz
    start = rospy.get_rostime()
    while not rospy.is_shutdown():
        # hello_str = "hello world %s" % start.secs
        # rospy.loginfo(hello_str)
        threshold = 0.4 if (rospy.get_rostime().secs - start.secs) < 1200 or (rospy.get_rostime().secs - start.secs) > 1500 else 0.8
        pub.publish(threshold)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass