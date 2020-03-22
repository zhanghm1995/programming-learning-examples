/*======================================================================
* Author   : Haiming Zhang
* Email    : zhanghm_1995@qq.com
* Version  :　2019年1月5日
* Copyright    :
* Descriptoin  : Learn how to create new Display Type Plugins for RViz
* References   :  http://docs.ros.org/kinetic/api/rviz_plugin_tutorials/html/display_plugin_tutorial.html
======================================================================*/

#include <rviz/visualization_manager.h>
#include <rviz/frame_manager.h>

#include "object_array_display.h"

namespace rviz_plugin_tutorials
{

// BEGIN_TUTORIAL
// The constructor must have no arguments, so we can't give the
// constructor the parameters it needs to fully initialize.
ObjectArrayDisplay::ObjectArrayDisplay()
  : rviz::MarkerDisplay()
{
  marker_topic_property_->setName("Topic");
  marker_topic_property_->setValue( "object_array_message" );
  marker_topic_property_->setMessageType(QString::fromStdString(ros::message_traits::datatype<rviz_msgs::ObjectArray>()));
  marker_topic_property_->setDescription("rviz_msgs::ObjectArray topic to subscribe to.");

  is_show_center_ = new rviz::BoolProperty("Show Center", true, 
                                                                                             "Whether or not show object centers",
                                                                                             this, SLOT(updateShowCenter()));
  is_show_bboxes_ = new rviz::BoolProperty("Show BBoxes", false, 
                                                                                              "Whether or not show object bounding box", this, SLOT(updateShowBBoxes()));
}

// After the top-level rviz::Display::initialize() does its own setup,
// it calls the subclass's onInitialize() function.  This is where we
// instantiate all the workings of the class.  We make sure to also
// call our immediate super-class's onInitialize() function, since it
// does important stuff setting up the message filter.
//
//  Note that "MFDClass" is a typedef of
// ``MessageFilterDisplay<message type>``, to save typing that long
// templated class name every time you need to refer to the
// superclass.
void ObjectArrayDisplay::onInitialize()
{
  rviz::MarkerDisplay::onInitialize();
}

void ObjectArrayDisplay::updateTopic()
{
  unsubscribe();
  reset();
  subscribe();
  // context_->queueRender();
}

void ObjectArrayDisplay::updateShowBBoxes()
{
  
}

void ObjectArrayDisplay::updateShowCenter()
{
}


ObjectArrayDisplay::~ObjectArrayDisplay()
{
  unsubscribe();
}

void ObjectArrayDisplay::subscribe()
{
  if (!isEnabled())
  {
    return;
  }

  try
  {
    unsubscribe();

    const std::string& topicStr = marker_topic_property_->getStdString();

    if (!topicStr.empty())
    {

      sub_.reset(new message_filters::Subscriber<rviz_msgs::ObjectArray>());

      sub_->subscribe(threaded_nh_, topicStr, 10);
      sub_->registerCallback(boost::bind(&ObjectArrayDisplay::handleObjectArrayMessage, this, _1));
    }
  }
  catch (ros::Exception& e)
  {
    setStatus(rviz::StatusProperty::Error, "Topic", (std::string("Error subscribing: ") + e.what()).c_str());
  }
}

void ObjectArrayDisplay::handleObjectArrayMessage(const rviz_msgs::ObjectArray::ConstPtr& msg)
{
  ROS_WARN("Enter in handleObjectArrayMessage");
  ROS_WARN("Enter in MyImuDisplay::processMessage function...");
  // 1) From rviz_msgs::ObjectArray message to visualization_msgs::MarkerArray message
  visualization_msgs::Marker centers;
  centers.header.frame_id = "base_link";
  centers.header.stamp = msg->header.stamp;
  centers.ns = "prefusion";
  centers.action = visualization_msgs::Marker::ADD;
  centers.pose.orientation.w = 1.0;
  centers.id = 0;
  centers.type = visualization_msgs::Marker::POINTS;
  centers.scale.x = 0.2;
  centers.scale.y = 0.2;
  centers.scale.z = 0.2;
  // Points are green
  centers.color.g = 1.0f;
  centers.color.a = 1.0;
  // Create the vertices for the centers
  static float f = 0.0;
  for (uint32_t i = 0; i < 100; ++i)
  {
    float y = 5 * sin(f + i / 100.0f * 2 * M_PI);
    float z = 5 * cos(f + i / 100.0f * 2 * M_PI);

    geometry_msgs::Point p;
    p.x = (int32_t)i - 50;
    p.y = y;
    p.z = z;

    centers.points.push_back(p);
  }
  f += 0.04;
  visualization_msgs::MarkerArray::Ptr marker_array_ptr(new visualization_msgs::MarkerArray());
  marker_array_ptr->markers.push_back(centers);
  // 2) Use the ObjectArrayDisplay to show the markers
  rviz::MarkerDisplay::incomingMarkerArray(marker_array_ptr);
}

void ObjectArrayDisplay::unsubscribe()
{
  try
  {
    // reset filters
    sub_.reset();
  }
  catch (ros::Exception& e)
  {
    setStatus(rviz::StatusProperty::Error, "Topic", (std::string("Error unsubscribing: ") + e.what()).c_str());
  }
}

} // end namespace rviz_plugin_tutorials

// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_plugin_tutorials::ObjectArrayDisplay,rviz::Display )
// END_TUTORIAL
