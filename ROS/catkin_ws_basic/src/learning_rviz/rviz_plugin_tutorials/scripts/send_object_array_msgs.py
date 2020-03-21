#!/usr/bin/env python

from rviz_msgs.msg import ObjectArray
import rospy
from math import cos, sin
import tf

topic = 'test_object_array'
publisher = rospy.Publisher( topic, ObjectArray, queue_size=10 )

rospy.init_node( 'test_object_array' )

rate = rospy.Rate(10)

dist = 3
while not rospy.is_shutdown():
    object_array = ObjectArray()

    # imu = Imu()
    # imu.header.frame_id = "/base_link"
    # imu.header.stamp = rospy.Time.now()
   
    # imu.linear_acceleration.x = sin( 10 * angle )
    # imu.linear_acceleration.y = sin( 20 * angle )
    # imu.linear_acceleration.z = sin( 40 * angle )

    publisher.publish( object_array )
    rate.sleep()

