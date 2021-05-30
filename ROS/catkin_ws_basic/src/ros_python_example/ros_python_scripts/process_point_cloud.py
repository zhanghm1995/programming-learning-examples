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


def convert_pc_numpy_to_msg(pc_data):
    cloud_arr = np.zeros(len(pc_data), dtype=([
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('intensity', np.float32)]))
         
    cloud_arr['x'] = pc_data[:, 0]
    cloud_arr['y'] = pc_data[:, 1]
    cloud_arr['z'] = pc_data[:, 2]
    cloud_arr['intensity'] = pc_data[:, 3]

    timestamp = rospy.Time.now()

    msg = ros_numpy.point_cloud2.array_to_pointcloud2(cloud_arr, timestamp, 'base_link')
    return msg

def publish_point_cloud():
    lidar_bin_file_dir = '/media/zhanghm/Data/Datasets/KITTI/tracking/training/velodyne/0000'
    file_list = read_point_cloud(lidar_bin_file_dir)

    rate = rospy.Rate(10)
    for file in file_list:
        if rospy.is_shutdown():
            break
        
        file_path = os.path.join(lidar_bin_file_dir, file)
        pc_data = np.fromfile(file_path, dtype=np.float32).reshape(-1, 4)
 
        msg = convert_pc_numpy_to_msg(pc_data)
        g_pub_pc.publish(msg)
        rate.sleep()
        print(pc_data.shape)


def init_ros():
    global g_pub_pc
    g_pub_pc = rospy.Publisher('/kitti/point_cloud', PointCloud2, queue_size=10)
    rospy.init_node('ros_process_point_cloud')


if __name__ == "__main__":
    init_ros()

    publish_point_cloud()

    rospy.spin()


    