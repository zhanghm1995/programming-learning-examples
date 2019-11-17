/*======================================================================
* Author   : Haiming Zhang
* Email    : zhanghm_1995@qq.com
* Version  :　2019年11月16日
* Copyright    :
* Descriptoin  : Learn how to subsribe image and publish it
* References   :
======================================================================*/

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h> //image handler
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/opencv.hpp>

namespace enc = sensor_msgs::image_encodings;

ros::Publisher g_pub_image;

void ImageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
  ROS_INFO("[ImageCallback] Received image...");
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
  cv::Mat image_receive = cv_ptr->image;
  cv::imshow("image_receive", image_receive);
  cv::waitKey(5);
}

void PublishImage()
{
  //publish depth image to ROS Image msg
  cv::Mat depth_image;
  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", depth_image).toImageMsg();
  g_pub_image.publish(msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_subsriber_publisher_node");
  ros::NodeHandle nh;
  ros::Subscriber sub_image = nh.subscribe("/kitti/camera_color_left/image_raw", 2, ImageCallback);//subscribe depth image
  g_pub_image = nh.advertise<sensor_msgs::Image>("/camera_image", 2);

  ros::spin();

  return 0;
}