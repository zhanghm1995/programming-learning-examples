#! /usr/bin/env python
import rospy
from std_msgs.msg import String

def talker():
    """https://github.com/ros/ros_tutorials/tree/noetic-devel/rospy_tutorials
    """
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker')
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass