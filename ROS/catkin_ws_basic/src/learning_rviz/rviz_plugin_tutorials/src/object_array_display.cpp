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
