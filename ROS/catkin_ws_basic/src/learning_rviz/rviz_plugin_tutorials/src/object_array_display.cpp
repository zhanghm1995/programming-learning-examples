/*======================================================================
* Author   : Haiming Zhang
* Email    : zhanghm_1995@qq.com
* Version  :　2019年1月5日
* Copyright    :
* Descriptoin  : Learn how to create new Display Type Plugins for RViz
* References   :  http://docs.ros.org/kinetic/api/rviz_plugin_tutorials/html/display_plugin_tutorial.html
======================================================================*/

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <tf/transform_listener.h>

#include <rviz/visualization_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/frame_manager.h>

#include "object_array_display.h"

namespace rviz_plugin_tutorials
{

// BEGIN_TUTORIAL
// The constructor must have no arguments, so we can't give the
// constructor the parameters it needs to fully initialize.
ObjectArrayDisplay::ObjectArrayDisplay()
{
  color_property_ = new rviz::ColorProperty( "Color", QColor( 204, 51, 204 ),
                                             "Color to draw the acceleration arrows.",
                                             this, SLOT( updateColorAndAlpha() ));
  is_show_bboxes_ = new rviz::BoolProperty("Show BBoxes", false, 
                                                                                              "Whether or not show object bounding box", this, SLOT(updateShowBBoxes));
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
  MFDClass::onInitialize();
}

void updateShowBBoxes()
{
  
}

ObjectArrayDisplay::~ObjectArrayDisplay()
{
}

// Clear the visuals by deleting their objects.
void ObjectArrayDisplay::reset()
{
  MFDClass::reset();
}

// Set the current color and alpha values for each visual.
void ObjectArrayDisplay::updateColorAndAlpha()
{
}

// This is our callback to handle an incoming message.
void ObjectArrayDisplay::processMessage( const rviz_msgs::ObjectArray::ConstPtr& msg )
{
}

} // end namespace rviz_plugin_tutorials

// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_plugin_tutorials::ObjectArrayDisplay,rviz::Display )
// END_TUTORIAL
