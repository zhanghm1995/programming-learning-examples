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
geometry_msgs::TransformStamped getStaticTransform(
    string target_frame_id, string source_frame_id, const Eigen::MatrixXf& transform)
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

const double ARC_TO_DEG = 57.29577951308238;
const double DEG_TO_ARC = 0.0174532925199433;

/**
 * @ref https://zhuanlan.zhihu.com/p/55790406
 */ 
static void toEulerAngle(const Eigen::Quaterniond& q, double& roll, double& pitch, double& yaw)
{
  // roll (x-axis rotation)
  double sinr_cosp = +2.0 * (q.w() * q.x() + q.y() * q.z());
  double cosr_cosp = +1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
  roll = atan2(sinr_cosp, cosr_cosp);

  // pitch (y-axis rotation)
  double sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
  if (fabs(sinp) >= 1)
    pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
  else
    pitch = asin(sinp);

  // yaw (z-axis rotation)
  double siny_cosp = +2.0 * (q.w() * q.z() + q.x() * q.y());
  double cosy_cosp = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
  yaw = atan2(siny_cosp, cosy_cosp);
}

/**
 * @brief 生成不同组的RPY角度值,用来进行不同情况的测试验证
 *                修改number值来修改
 *                角度单位: radian
 */ 
void GenerateRPY(double& roll, double& pitch, double& yaw)
{
  const int number = 3;

  if (number == 1) {
    // 这一组会在Eigen::Quaterniond转为Euler角时出现歧义
    roll = 0.22857;
    pitch = 0.004342;
    yaw = -1.20607667;
  } else if (number == 2) {
   double roll_deg = 0.5;      // 绕X轴
   double pitch_deg = 0.8;     // 绕Y轴
   double yaw_deg = 108.5;     // 绕Z轴

   // 转化为弧度
   roll = roll_deg * DEG_TO_ARC;    // 绕X轴
   pitch = pitch_deg * DEG_TO_ARC;  // 绕Y轴
   yaw = yaw_deg * DEG_TO_ARC;      // 绕Z轴
  } else if (number == 3) {
    roll = 0.0;
    pitch = 0.0;
    yaw = 0.7853981633974483; // PI / 4
  } else {
    cout<<"Not implemented"<<endl;
  }
}

void TransformInROS(const double roll, const double pitch, const double yaw)
{
  cout<<"===============Begin TransformInROS======================="<<endl;
  // 从欧拉角转四元数
  tf::Quaternion q = tf::createQuaternionFromRPY(roll, pitch, yaw);
  // 或者用
  q.setRPY(roll, pitch, yaw);
  cout<< "tf QUaternion "<<endl<<q.getX()<<" "<<q.getY()<<" "<<q.getZ()<<" "<<q.getW()<<endl;
  cout<<"tf yaw "<<endl<<tf::getYaw(q)<<endl;
  cout<<"===============End TransformInROS======================="<<endl;
}

void TransformInEigen(const double roll, const double pitch, const double yaw)
{
  cout<<"===============Begin TransformInEigen======================="<<endl;
  // 从欧拉角转四元数
  Eigen::Quaterniond q_eigen;
  q_eigen =  Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *  
                        Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) * 
                        Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
  std::cout << "Eigen Quaternion" << std::endl << q_eigen.coeffs() << std::endl; // 打印Eigen::Quaternion内容

  // Note: 从四元数到欧拉角, 有可能存在角度的歧义性, 用toEulerAngle函数可以解决
  Eigen::Vector3d euler_angles = q_eigen.matrix().eulerAngles(2, 1, 0); // ZYX顺序, 得到yaw, pitch, roll
  cout<<"Eigen yaw pitch roll from Quaternion:"<<endl<<euler_angles<<endl;
  double roll_tmp, pitch_tmp, yaw_tmp;
  toEulerAngle(q_eigen, roll_tmp, pitch_tmp, yaw_tmp);
  cout<<"经过转换后的RPY: "<<roll_tmp<<" "<<pitch_tmp<<" "<<yaw_tmp<<" "<<endl;

  cout<<"==========欧拉角转四元数是唯一的========"<<endl;
  q_eigen = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *  
                       Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) * 
                       Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
  std::cout << "Eigen Quaternion" << std::endl << q_eigen.coeffs() << std::endl;

  // 四元数转旋转矩阵
  Eigen::Matrix3d rot = q_eigen.matrix();
  cout<<"Eigen rotation matrix"<<endl<<rot<<endl;
  cout<<"roation matrix inverse is its transpose: "<<endl<<rot.inverse()<<endl;

  // 旋转矩阵到四元数
  Eigen::Quaterniond qq_eign(rot);
  std::cout << "Eigen Quaternion from rotation matrix: " << std::endl << qq_eign.coeffs() << std::endl;

  cout<<"===============End TransformInEigen======================="<<endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_node");
  ros::NodeHandle nh;

  // 测试ROS tf中从roll, pitch, yaw到旋转矩阵的转换和Eigen的转换的区别
  cout<<"*********************************欧拉角与tf四元数和Eigen四元数*********************************************"<<endl;
  double roll, pitch, yaw;
  GenerateRPY(roll, pitch, yaw);
  cout<<"原始的RPY: "<<roll<<" "<<pitch<<" "<<yaw<<endl;

  TransformInROS(roll, pitch, yaw);

  TransformInEigen(roll, pitch, yaw);
  

  cout<<"-------测试ROS tf中的坐标变换--------"<<endl;
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
  cout<<"获得velo_link相对camera_color_left坐标系的变换关系"<<endl;
  cout<<transform.getOrigin().getX()<<" "
           <<transform.getOrigin().getY()<<" "
           <<transform.getOrigin().getZ()<<endl;

  // 打印旋转矩阵
  auto T =  transform;
  printf("[ %f %f %f \n %f %f %f \n %f %f %f ]\n", 
                T.getBasis()[0][0], T.getBasis()[0][1], T.getBasis()[0][2],
                T.getBasis()[1][0], T.getBasis()[1][1], T.getBasis()[1][2],
                T.getBasis()[2][0], T.getBasis()[2][1], T.getBasis()[2][2]);

  Eigen::Quaterniond qq(transform.getRotation().getW(), transform.getRotation().getX(),
                                                  transform.getRotation().getY(),transform.getRotation().getZ());
  cout<<"从四元数得到的rotation "<<endl<<qq.matrix()<<endl;

  ros::spin();

  return 0;
}