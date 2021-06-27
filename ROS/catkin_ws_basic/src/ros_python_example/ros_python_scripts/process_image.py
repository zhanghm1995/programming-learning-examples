# -*- coding:utf8 -*-
import sys
if '/opt/ros/kinetic/lib/python2.7/dist-packages' in sys.path:
    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')

# import sys
# sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
# 或者
# export PYTHONPATH=""

import os
import time
from cv_bridge import CvBridge
import math
import numpy as np
from std_msgs.msg import Header
from sensor_msgs.msg import Image
import cv2
import rospy

def init_ros():
    global pub_img
    pub_img = rospy.Publisher('/kitti/image_2', Image, queue_size=10)
    rospy.init_node('process_image')

def convert_img2msg(imgdata):
    """Publish the image message to ROS in Python2 environment.
    """
    _bridge = CvBridge()
    ros_msg_image = _bridge.cv2_to_imgmsg(imgdata, "bgr8")
    pub_img.publish(ros_msg_image)

def publish_image(imgdata):
    """ Publish the image message to ROS in Python3 environment. 
    Ref: https://blog.csdn.net/lucky__ing/article/details/79949294
    """
    IMAGE_HEIGHT = imgdata.shape[0]
    IMAGE_WIDTH = imgdata.shape[1]
    IMAGE_CHANNELS = imgdata.shape[2]
    
    image_temp=Image()
    header = Header(stamp=rospy.Time.now())
    header.frame_id = 'base_link'
    image_temp.height = IMAGE_HEIGHT
    image_temp.width = IMAGE_WIDTH
    image_temp.encoding = 'bgr8'
    image_temp.data=np.array(imgdata).tostring()
    image_temp.header = header
    image_temp.step= IMAGE_WIDTH * IMAGE_CHANNELS
    pub_img.publish(image_temp)


if __name__ == '__main__':
    init_ros()
    
    start_time = time.time()
    dir_path = '/media/zhanghm/Data/Datasets/KITTI/tracking/training/image_02/0000'
    name_list = os.listdir(dir_path)
    name_list.sort()

    for file in name_list:
        path = os.path.join(dir_path, file)
        print(path)
        img = cv2.imread(path)
        cv2.imshow('img', img)
        cv2.waitKey(10)
        print(img.shape)
        publish_image(img)
        time.sleep(0.1)

        
    print("execution time: {}s".format(time.time() - start_time))

