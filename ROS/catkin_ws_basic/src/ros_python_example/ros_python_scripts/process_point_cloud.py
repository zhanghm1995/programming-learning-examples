import os

import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2
import ros_numpy # sudo apt-get install ros-$release-ros-numpy

# inline void CloudInObject2PointCloud2Msg(
#         const LidarObject::BBoxVec& in,
#         const std_msgs::Header& header,
#         sensor_msgs::PointCloud2& out)
# {
#     /// Get the total points number
#     int points_num =
#             std::accumulate(in.begin(), in.end(), 0,
#                             [](int a, const LidarObject::BBox& b) {
#                                 return a + b.cloud->size();
#                             });

#     /// Build sensor_msgs::PointCloud2 message
#     out.header = header;
#     out.width = points_num;
#     sensor_msgs::PointCloud2Modifier modifier(out);
#     modifier.setPointCloud2Fields(4,
#                                   "x", 1, sensor_msgs::PointField::FLOAT32,
#                                   "y", 1, sensor_msgs::PointField::FLOAT32,
#                                   "z", 1, sensor_msgs::PointField::FLOAT32,
#                                   "rgb", 1, sensor_msgs::PointField::FLOAT32);
#     modifier.resize(points_num);
#     sensor_msgs::PointCloud2Iterator<float> iter_x(out, "x");
#     sensor_msgs::PointCloud2Iterator<float> iter_y(out, "y");
#     sensor_msgs::PointCloud2Iterator<float> iter_z(out, "z");
#     sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(out, "r");
#     sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(out, "g");
#     sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(out, "b");

#     for (const auto& iter : in) {
#         for (const auto& pt : iter.cloud->points) {
#             *iter_x = pt.x;
#             *iter_y = pt.y;
#             *iter_z = pt.z;
#             *iter_r = 255;
#             *iter_g = 255;
#             *iter_b = 0;
#             ++iter_x;
#             ++iter_y;
#             ++iter_z;
#             ++iter_r;
#             ++iter_g;
#             ++iter_b;
#         }
#     }
# }


def read_point_cloud(lidar_bin_file_dir):
    file_list = os.listdir(lidar_bin_file_dir)
    file_list.sort()
    
    return file_list


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


def publish_one_pc(ros_pub, file_path, load_dim=3, use_dim=3):
    if isinstance(use_dim, int):
        use_dim = list(range(use_dim))

    pc_data = np.fromfile(file_path, dtype=np.float32).reshape(-1, load_dim)
    pc_data = pc_data[:, use_dim]

    msg = convert_pc_numpy_to_msg(pc_data)
    ros_pub.publish(msg)


def publish_point_cloud():
    lidar_bin_file_dir = '/media/zhanghm/Data/Datasets/KITTI/tracking/training/velodyne/0000'
    file_list = read_point_cloud(lidar_bin_file_dir)

    rate = rospy.Rate(10)
    field_properties = [
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('intensity', np.float32)]

    for file in file_list:
        if rospy.is_shutdown():
            break
        
        file_path = os.path.join(lidar_bin_file_dir, file)
        pc_data = np.fromfile(file_path, dtype=np.float32).reshape(-1, 4)

        msg = convert_pc_numpy_to_msg(pc_data, field_properties)
        g_pub_pc.publish(msg)
        rate.sleep()
        print(pc_data.shape)


def publish_point_cloud_v2(pt_numpy):
    """Publish points in numpy matrix format to show the points location in Rviz
    pt_numpy: [N, 3]
    """
    field_properties = [
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32)]
    
    rate = rospy.Rate(10)
    while(True):
        if rospy.is_shutdown():
            break
        msg = convert_pc_numpy_to_msg(pt_numpy, field_properties)
        g_pub_pc.publish(msg)
        rate.sleep()


def init_ros():
    """Init the ROS node and publishers and subscribers
    """
    global g_pub_pc
    g_pub_pc = rospy.Publisher('/kitti/point_cloud', PointCloud2, queue_size=10)
    rospy.init_node('ros_process_point_cloud')


if __name__ == "__main__":
    init_ros()

    # publish_point_cloud()

    from generate_object import generate_data
    pt_numpy = generate_data(10)
    z = np.zeros(pt_numpy.shape[0])
    pt_numpy = np.c_[pt_numpy, z]
    print(pt_numpy)
    publish_point_cloud_v2(pt_numpy)
    