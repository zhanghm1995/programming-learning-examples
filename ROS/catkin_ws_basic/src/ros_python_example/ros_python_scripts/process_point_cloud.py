import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2


inline void CloudInObject2PointCloud2Msg(
        const LidarObject::BBoxVec& in,
        const std_msgs::Header& header,
        sensor_msgs::PointCloud2& out)
{
    /// Get the total points number
    int points_num =
            std::accumulate(in.begin(), in.end(), 0,
                            [](int a, const LidarObject::BBox& b) {
                                return a + b.cloud->size();
                            });

    /// Build sensor_msgs::PointCloud2 message
    out.header = header;
    out.width = points_num;
    sensor_msgs::PointCloud2Modifier modifier(out);
    modifier.setPointCloud2Fields(4,
                                  "x", 1, sensor_msgs::PointField::FLOAT32,
                                  "y", 1, sensor_msgs::PointField::FLOAT32,
                                  "z", 1, sensor_msgs::PointField::FLOAT32,
                                  "rgb", 1, sensor_msgs::PointField::FLOAT32);
    modifier.resize(points_num);
    sensor_msgs::PointCloud2Iterator<float> iter_x(out, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(out, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(out, "z");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(out, "r");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(out, "g");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(out, "b");

    for (const auto& iter : in) {
        for (const auto& pt : iter.cloud->points) {
            *iter_x = pt.x;
            *iter_y = pt.y;
            *iter_z = pt.z;
            *iter_r = 255;
            *iter_g = 255;
            *iter_b = 0;
            ++iter_x;
            ++iter_y;
            ++iter_z;
            ++iter_r;
            ++iter_g;
            ++iter_b;
        }
    }
}



if __name__ == "__main__":
    init_ros()

    publish_point_cloud()


    