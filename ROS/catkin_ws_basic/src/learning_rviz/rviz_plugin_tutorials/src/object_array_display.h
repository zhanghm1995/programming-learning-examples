/*======================================================================
* Author   : Haiming Zhang
* Email    : zhanghm_1995@qq.com
* Version  :　2019年1月5日
* Copyright    :
* Descriptoin  : Learn how to create new Display Type Plugins for RViz
* References   :  http://docs.ros.org/kinetic/api/rviz_plugin_tutorials/html/display_plugin_tutorial.html
======================================================================*/

#ifndef ObjectArray_DISPLAY_H
#define ObjectArray_DISPLAY_H


#ifndef Q_MOC_RUN
#include <rviz/message_filter_display.h>
#include <rviz/default_plugin/marker_display.h>
#include <rviz_msgs/ObjectArray.h>
#endif

namespace Ogre
{
class SceneNode;
}

namespace rviz
{
class BoolProperty;
}

// All the source in this plugin is in its own namespace.  This is not
// required but is good practice.
namespace rviz_plugin_tutorials
{

// BEGIN_TUTORIAL
// Here we declare our new subclass of rviz::Display.  Every display
// which can be listed in the "Displays" panel is a subclass of
// rviz::Display.
//
// ImuDisplay will show a 3D arrow showing the direction and magnitude
// of the IMU acceleration vector.  The base of the arrow will be at
// the frame listed in the header of the Imu message, and the
// direction of the arrow will be relative to the orientation of that
// frame.  It will also optionally show a history of recent
// acceleration vectors, which will be stored in a circular buffer.
//
// The ImuDisplay class itself just implements the circular buffer,
// editable parameters, and Display subclass machinery.  The visuals
// themselves are represented by a separate class, ImuVisual.  The
// idiom for the visuals is that when the objects exist, they appear
// in the scene, and when they are deleted, they disappear.
class ObjectArrayDisplay: public rviz::MarkerDisplay 
{
Q_OBJECT
public:
  // Constructor.  pluginlib::ClassLoader creates instances by calling
  // the default constructor, so make sure you have one.
  ObjectArrayDisplay();
  virtual ~ObjectArrayDisplay();

  // Overrides of protected virtual functions from Display.  As much
  // as possible, when Displays are not enabled, they should not be
  // subscribed to incoming data and should not show anything in the
  // 3D view.  These functions are where these connections are made
  // and broken.
protected:
  virtual void onInitialize();
  virtual void subscribe();
  virtual void unsubscribe();

  boost::shared_ptr<message_filters::Subscriber<rviz_msgs::ObjectArray> > sub_;

  void handleObjectArrayMessage(const rviz_msgs::ObjectArray::ConstPtr& msg);


  // These Qt slots get connected to signals indicating changes in the user-editable properties.
private Q_SLOTS:
  void updateTopic();
  void updateShowCenter();
  void updateShowBBoxes();

  // Function to handle an incoming ROS message.
private:
  // User-editable property variables.
  rviz::BoolProperty* is_show_center_;
  rviz::BoolProperty* is_show_bboxes_;
};
// END_TUTORIAL

} // end namespace rviz_plugin_tutorials

#endif // ObjectArray_DISPLAY_H
// %EndTag(FULL_SOURCE)%