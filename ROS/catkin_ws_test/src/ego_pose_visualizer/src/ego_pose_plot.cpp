/*======================================================================
* Author   : Haiming Zhang
* Email    : zhanghm_1995@qq.com
* Version  :　2019年4月19日
* Copyright    :
* Descriptoin  : Using Rviz to show ego vehicle pose trajectory
* References   :
======================================================================*/

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <sensor_driver_msgs/GpswithHeading.h>


/// Cartesian coordinates struct, refs# 522
struct Xy
{
    double x;
    double y;
};

/** Conversion between geographic and UTM coordinates
    Adapted from:  http://www.uwgb.edu/dutchs/UsefulData/ConvertUTMNoOZ.HTM
    Refs# 522
**/
Xy latlon2xy_helper(double lat, double lngd)
{
    // WGS 84 datum
    double eqRad = 6378137.0;
    double flat = 298.2572236;

    // constants used in calculations:
    double a = eqRad;           // equatorial radius in meters
    double f = 1.0 / flat;        // polar flattening
    double b = a * (1.0 - f);     // polar radius
    double e = sqrt(1.0 - (pow(b, 2) / pow(a, 2))); // eccentricity
    double k0 = 0.9996;
    double drad = M_PI / 180.0;

    double phi = lat * drad;   // convert latitude to radians
    double utmz = 1.0 + floor((lngd + 180.0) / 6.0); // longitude to utm zone
    double zcm = 3.0 + 6.0 * (utmz - 1.0) - 180.0;     // central meridian of a zone
    double esq = (1.0 - (b / a) * (b / a));
    double e0sq = e * e / (1.0 - e * e);
    double M = 0.0;
    double M0 = 0.0;
    double N = a / sqrt(1.0 - pow(e * sin(phi), 2));
    double T = pow(tan(phi), 2);
    double C = e0sq * pow(cos(phi), 2);
    double A = (lngd - zcm) * drad * cos(phi);

    // calculate M (USGS style)
    M = phi * (1.0 - esq * (1.0 / 4.0 + esq * (3.0 / 64.0 + 5.0 * esq / 256.0)));
    M = M - sin(2.0 * phi) * (esq * (3.0 / 8.0 + esq * (3.0 / 32.0 + 45.0 * esq / 1024.0)));
    M = M + sin(4.0 * phi) * (esq * esq * (15.0 / 256.0 + esq * 45.0 / 1024.0));
    M = M - sin(6.0 * phi) * (esq * esq * esq * (35.0 / 3072.0));
    M = M * a; // Arc length along standard meridian

    // now we are ready to calculate the UTM values...
    // first the easting (relative to CM)
    double x = k0 * N * A * (1.0 + A * A * ((1.0 - T + C) / 6.0 + A * A * (5.0 - 18.0 * T + T * T + 72.0 * C - 58.0 * e0sq) / 120.0));
    x = x + 500000.0; // standard easting

    // now the northing (from the equator)
    double y = k0 * (M - M0 + N * tan(phi) * (A * A * (1.0 / 2.0 + A * A * ((5.0 - T + 9.0 * C + 4.0 * C * C) / 24.0 + A * A * (61.0 - 58.0 * T + T * T + 600.0 * C - 330.0 * e0sq) / 720.0))));
    if (y < 0)
    {
        y = 10000000.0 + y; // add in false northing if south of the equator
    }
    double easting  = x;
    double northing = y;

    Xy coords;
    coords.x = easting;
    coords.y = northing;

    return coords;
}

inline double deg2rad(double degree)
{
  double pi = 3.1415926;
  double res = (degree * pi) / 180.0;
  return res;
}

/// --------Global variables----------
// Publish path
ros::Publisher path_pub;
nav_msgs::Path path;
static long long count = 0;


void callbackGPSwithHeading(const sensor_driver_msgs::GpswithHeading::ConstPtr& gpsmsg)
{
  ++ count;

  static double* origin = nullptr;
  // Get translation
  Xy tr = latlon2xy_helper(gpsmsg->gps.latitude, gpsmsg->gps.longitude);

  if(origin == nullptr) {
    origin = new double[3];
    origin[0] = tr.x;
    origin[1] = tr.y;
    origin[2] = gpsmsg->gps.altitude;
  }

  // Get rotation
  //    - roll:    roll angle (rad),  0 = level, positive = left side up (-pi..pi)
  //    - pitch:   pitch angle (rad), 0 = level, positive = front down (-pi/2..pi/2)
  //    - yaw:     heading (rad),     0 = east,  positive = counter clockwise (-pi..pi)
  // Change the heading theta
  double heading_new = -gpsmsg->heading + 90.0;
  if (heading_new > 180.0) {
    heading_new -= 360.0;
  }
  tf::Quaternion q = tf::createQuaternionFromRPY(deg2rad(gpsmsg->roll), deg2rad(gpsmsg->pitch), deg2rad(heading_new));

  geometry_msgs::PoseStamped this_pose_stamped;
  this_pose_stamped.pose.position.x = tr.x - origin[0];
  this_pose_stamped.pose.position.y = tr.y - origin[1];
//  this_pose_stamped.pose.position.z = gpsmsg->gps.altitude - origin[2];
  this_pose_stamped.pose.position.z = 0.0;

  //    geometry_msgs::Quaternion goal_quat = transformStamped.transform.rotation;
  this_pose_stamped.pose.orientation.x = q.x();
  this_pose_stamped.pose.orientation.y = q.y();
  this_pose_stamped.pose.orientation.z = q.z();
  this_pose_stamped.pose.orientation.w = q.w();

  this_pose_stamped.header.stamp = gpsmsg->header.stamp;
  this_pose_stamped.header.frame_id = "world";

  ROS_WARN("[%d] Get gpsdata topic x:%.3f y:%.3f heading:%.3f......", count,
            this_pose_stamped.pose.position.x,
            this_pose_stamped.pose.position.y,
            gpsmsg->heading);
  // Publish nav_msgs
  path.header.stamp = this_pose_stamped.header.stamp;
  path.header.frame_id = "world";
  path.poses.push_back(this_pose_stamped);
  path_pub.publish(path);// publish nav_msgs::Path

  /// -----------publish tf----------
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;
  transformStamped.header.stamp = gpsmsg->header.stamp;
  transformStamped.header.frame_id = "world";
  transformStamped.child_frame_id = "base_link";
  transformStamped.transform.translation.x = this_pose_stamped.pose.position.x;
  transformStamped.transform.translation.y = this_pose_stamped.pose.position.y;
  transformStamped.transform.translation.z = this_pose_stamped.pose.position.z;

  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();
  br.sendTransform(transformStamped);
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "path_plot_node");
  ros::NodeHandle n;
  ros::Rate r(100);
  path_pub = n.advertise<nav_msgs::Path>("trajectory", 1);

  ros::Subscriber gpsdata_sub = n.subscribe("/gpsdata", 10, callbackGPSwithHeading);

  ros::spin();
}

