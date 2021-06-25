/*======================================================================
* Author   : Haiming Zhang
* Email    : zhanghm_1995@qq.com
* Version  :
* Copyright    :
* Descriptoin  : 发布static_tf消息
* References   :
======================================================================*/

// C++
#include <string>
// Eigen
#include <Eigen/Dense>
// ROS
#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_listener.h>

geometry_msgs::TransformStamped getStaticTransform(
    std::string target_frame_id, std::string source_frame_id, const Eigen::MatrixXf& transform)
{
    geometry_msgs::TransformStamped static_transformStamped;
    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id = target_frame_id;
    static_transformStamped.child_frame_id = source_frame_id;

    Eigen::Matrix3f Rot = transform.topLeftCorner(3,3);
    Eigen::VectorXf Tr = transform.rightCols(1);
    Eigen::Quaternionf q(Rot);

    static_transformStamped.transform.translation.x = Tr[0];
    static_transformStamped.transform.translation.y = Tr[1];
    static_transformStamped.transform.translation.z = Tr[2];
    static_transformStamped.transform.rotation.x = q.x();
    static_transformStamped.transform.rotation.y = q.y();
    static_transformStamped.transform.rotation.z = q.z();
    static_transformStamped.transform.rotation.w = q.w();
    return static_transformStamped;
}

int main(int argc, char** argv)
{
    // Begin publish static tf
    Eigen::Matrix4f RT_velo_to_cam, RT_imu_to_velo, RT_velo_to_base;
    RT_velo_to_cam<<7.533745e-03,-9.999714e-01,-6.166020e-04,-4.069766e-03,
                    1.480249e-02,7.280733e-04,-9.998902e-01, -7.631618e-02,
                    9.998621e-01,7.523790e-03,1.480755e-02,  -2.717806e-01,
                    0.0, 0.0, 0.0, 1.0;
    RT_imu_to_velo<<9.999976e-01, 7.553071e-04, -2.035826e-03, -8.086759e-01,
                    -7.854027e-04,9.998898e-01, -1.482298e-02, 3.195559e-01,
                    2.024406e-03, 1.482454e-02, 9.998881e-01,-7.997231e-01,
                    0.0, 0.0, 0.0, 1.0;
    RT_velo_to_base<<1.0, 0.0, 0.0, 0.0,
                     0.0,1.0, 0.0, 0.0,
                     0.0, 0.0, 1.0, 1.73,
                     0.0, 0.0, 0.0, 1.0;

    std::vector<geometry_msgs::TransformStamped> static_transformStamped_vec;
    geometry_msgs::TransformStamped static_transform_velo2base = 
        getStaticTransform("base_link", "velo_link", RT_velo_to_base);
    geometry_msgs::TransformStamped static_transform_cam2velo =
        getStaticTransform("velo_link", "camera_color_left", RT_velo_to_cam.inverse());

    static_transformStamped_vec.push_back(static_transform_velo2base);
    static_transformStamped_vec.push_back(static_transform_cam2velo);
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    static_broadcaster.sendTransform(static_transformStamped_vec);
    return 0;
}