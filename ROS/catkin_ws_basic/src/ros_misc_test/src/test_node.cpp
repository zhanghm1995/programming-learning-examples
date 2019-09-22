/*======================================================================
* Author   : Haiming Zhang
* Email    : zhanghm_1995@qq.com
* Version  :　2018年12月24日
* Copyright    :
* Descriptoin  :
* References   :
======================================================================*/

#include <iostream>
#include <string>
#include <fstream>
#include <istream>

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include "pcl_ros/impl/transforms.hpp"

#include <tf/LinearMath/Matrix3x3.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_listener.h>

using namespace std;

ros::Time lidar_time;
ros::Publisher cloud_pub;
bool flag = true;
geometry_msgs::TransformStamped getStaticTransform(string target_frame_id, string source_frame_id,
                      const Eigen::MatrixXf& transform)
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

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
  static unsigned int count = 0;
  // Convert to pcl cloud
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_in(new pcl::PointCloud<pcl::PointXYZI>);
  sensor_msgs::PointCloud2Ptr cloudMsg(new sensor_msgs::PointCloud2(*cloud));
  cloudMsg->fields[3].name = "intensity";
  pcl::fromROSMsg(*cloudMsg, *pcl_in);

  pcl::PointCloud<pcl::PointXYZI> newCloud;

  tf::TransformListener listener;
  listener.waitForTransform("/base_link", "/velo_link", cloud->header.stamp, ros::Duration(1.0));
  pcl_ros::transformPointCloud("/base_link", *pcl_in, newCloud, listener);

  cloud_pub.publish(newCloud);
}

/**
 * @brief parseTime
 * @param timestamp in Epoch
 * @return std_msgs::Header with input timpestamp converted from file input
 *
 * Epoch time conversion
 * http://www.epochconverter.com/programming/functions-c.php
 */
std_msgs::Header parseTime(string timestamp)
{

    std_msgs::Header header;

    // example: 2011-09-26 13:21:35.134391552
    //          01234567891111111111222222222
    //                    0123456789012345678
    struct tm t = {0};  // Initalize to all 0's
    t.tm_year = boost::lexical_cast<int>(timestamp.substr(0, 4)) - 1900;
    t.tm_mon  = boost::lexical_cast<int>(timestamp.substr(5, 2)) - 1;
    t.tm_mday = boost::lexical_cast<int>(timestamp.substr(8, 2));
    t.tm_hour = boost::lexical_cast<int>(timestamp.substr(11, 2));
    t.tm_min  = boost::lexical_cast<int>(timestamp.substr(14, 2));
    t.tm_sec  = boost::lexical_cast<int>(timestamp.substr(17, 2));
    t.tm_isdst = -1;
    time_t timeSinceEpoch = mktime(&t);

    header.stamp.sec  = timeSinceEpoch;
    header.stamp.nsec = boost::lexical_cast<int>(timestamp.substr(20, 9));
    return header;
}

#if 0
int main(int argc, char** argv)
{
  int entries_played = 0;
  string str_support =  "/home/zhanghm/Datasets/KITTI/raw_data/2011_09_26/2011_09_26_drive_0005_sync/oxts/timestamps.txt";
  ifstream timestamps(str_support.c_str());
  if (!timestamps.is_open())
  {
    ROS_ERROR_STREAM("Fail to open " << str_support);
    return -1;
  }
  timestamps.seekg(30 * entries_played);
  getline(timestamps, str_support);
  std_msgs::Header my_header;
  my_header.stamp = parseTime(str_support).stamp;
  ROS_WARN_STREAM("oxts time is "<<setprecision(20)<<my_header.stamp.toNSec());
}
#endif


int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_node");
  ros::NodeHandle nh;

  // 测试ROS tf中从roll, pitch, yaw到旋转矩阵的转换和Eigen的转换的区别
  float roll,pitch,yaw;
  roll = 0.22857;
  pitch = 0.004342;
  yaw = -1.20607667;
  tf::Quaternion q = tf::createQuaternionFromRPY(roll, pitch, yaw);
  cout<< "tf QUaternion "<<endl<<q.getX()<<" "<<q.getY()<<" "<<q.getZ()<<" "<<q.getW()<<endl;
  cout<<"tf yaw "<<endl<<tf::getYaw(q)<<endl;

  Eigen::Quaterniond q_eigen;
  q_eigen =   Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
            *  Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
           * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
  std::cout << "Eigen Quaternion" << std::endl << q_eigen.coeffs() << std::endl;

  Eigen::Matrix3d rot = q_eigen.matrix();
  cout<<"Eigen rotation matrix "<<rot.rows()<<" "<<rot.cols()<<endl<<rot<<endl;

  Eigen::Quaterniond qq_eign(rot);
  std::cout << "Eigen Quaternion" << std::endl << qq_eign.coeffs() << std::endl;

  Eigen::Vector3d euler_angles = q_eigen.toRotationMatrix().eulerAngles ( 2,1,0 ); // ZYX顺序
  cout<<"yaw pitch roll = "<<euler_angles.transpose()<<endl;
  yaw = euler_angles[0];
  pitch = euler_angles[1];
  roll = euler_angles[2];
  cout<<"=================="<<endl;
  q_eigen =   Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
              *  Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
             * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
    std::cout << "Eigen Quaternion" << std::endl << q_eigen.coeffs() << std::endl;

  cout<<"---------------"<<endl;
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

  vector<geometry_msgs::TransformStamped> static_transformStamped_vec;
  geometry_msgs::TransformStamped static_transform_velo2base =
            getStaticTransform("base_link", "velo_link", RT_velo_to_base);
  geometry_msgs::TransformStamped static_transform_cam2velo =
      getStaticTransform("velo_link", "camera_color_left", RT_velo_to_cam.inverse());

  static_transformStamped_vec.push_back(static_transform_velo2base);
  static_transformStamped_vec.push_back(static_transform_cam2velo);
  static tf2_ros::StaticTransformBroadcaster static_broadcaster;
  static_broadcaster.sendTransform(static_transformStamped_vec);

  // Subscribe the static transform
  tf::TransformListener listener;
  tf::StampedTransform transform;
  listener.waitForTransform("/camera_color_left", "velo_link", ros::Time(0),ros::Duration(1.0));
  listener.lookupTransform("/camera_color_left", "/velo_link", ros::Time(0), transform);
  cout<<transform.getOrigin().getX()<<" "<<transform.getOrigin().getY()<<" "<<
      transform.getOrigin().getZ()<<endl;
  Eigen::Quaterniond qq(transform.getRotation().getW(), transform.getRotation().getX(),
      transform.getRotation().getY(),transform.getRotation().getZ());
  cout<<"rotation "<<endl<<qq.matrix()<<endl;

  ros::spin();
  return 0;
}

