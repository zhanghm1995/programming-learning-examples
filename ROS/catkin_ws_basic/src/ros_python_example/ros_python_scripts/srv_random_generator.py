import os

import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2
import ros_numpy # sudo apt-get install ros-$release-ros-numpy
from std_srvs.srv import Trigger, TriggerResponse, TriggerRequest


def convert_pc_numpy_to_msg(pc_data, field_properties):
    """
    Args:
        field_properties is a tuple list to define the Fields name and DataType in PointCloud2
        field_properties = [
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('intensity', np.float32),
            ('ring', np.int32)]
    """
    assert pc_data.shape[1] == len(field_properties),  \
        f'The size of PointCloud is {pc_data.shape(1)} is not equal to Field Properties size!'
    
    cloud_arr = np.zeros(len(pc_data), dtype=(field_properties))
    for i, name in enumerate(cloud_arr.dtype.names):
        cloud_arr[name] = pc_data[:, i]

    timestamp = rospy.Time.now()

    msg = ros_numpy.point_cloud2.array_to_pointcloud2(cloud_arr, timestamp, 'base_link')
    return msg


def publish_point_cloud_v2(pt_numpy):
    """Publish points in numpy matrix format to show the points location in Rviz
    pt_numpy: [N, 3]
    """
    field_properties = [
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32)]
    
    msg = convert_pc_numpy_to_msg(pt_numpy, field_properties)
    g_pub_pc.publish(msg)


def gen_random_point_cloud(request):
    print("gen_random_point_cloud")
    from generate_object import generate_data
    pt_numpy = generate_data(10)
    z = np.zeros(pt_numpy.shape[0])
    pt_numpy = np.c_[pt_numpy, z]
    print(pt_numpy)
    publish_point_cloud_v2(pt_numpy)
    return TriggerResponse(success=True)


def init_ros():
    """Init the ROS node and publishers and subscribers
    """
    global g_pub_pc
    rospy.init_node('random_generator_srv_node')

    g_pub_pc = rospy.Publisher('/kitti/point_cloud', PointCloud2, queue_size=10)
    s = rospy.Service('generate_random_center', Trigger, gen_random_point_cloud)


if __name__ == "__main__":
    init_ros()
    print("Server ready...")
    rospy.spin()
    