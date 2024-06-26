/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <tf/transform_listener.h>

#include <rviz/visualization_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/bool_property.h>
#include <rviz/frame_manager.h>

#include "my_imu_display.h"

namespace rviz_plugin_tutorials
{

// BEGIN_TUTORIAL
// The constructor must have no arguments, so we can't give the
// constructor the parameters it needs to fully initialize.
MyImuDisplay::MyImuDisplay()
{
  is_show_bboxes_ = new rviz::BoolProperty("Show BBoxes", true, 
                                                                                             "Whether or not show object bouding boxes",
                                                                                             this, SLOT(updateShowBBoxes()));
  is_show_center_ = new rviz::BoolProperty("Show Center", true, 
                                                                                             "Whether or not show object centers",
                                                                                             this, SLOT(updateShowCenter()));
  object_array_marker_display_ = new ObjectArrayMarkerDisplay();                                                                                                                                                             
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
void MyImuDisplay::onInitialize()
{
  MFDClass::onInitialize();
  // object_array_marker_display_->onInitialize();
}

MyImuDisplay::~MyImuDisplay()
{
}

// Clear the visuals by deleting their objects.
void MyImuDisplay::reset()
{
  MFDClass::reset();
}

void MyImuDisplay::updateShowBBoxes()
{

}

void MyImuDisplay::updateShowCenter()
{
  ROS_INFO("enter in MyImuDisplay::updateShowCenter");
}

// This is our callback to handle an incoming message.
void MyImuDisplay::processMessage( const rviz_msgs::ObjectArray::ConstPtr& msg )
{
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
  object_array_marker_display_->addMarkerArrayMsg(marker_array_ptr);
}

ObjectArrayMarkerDisplay::ObjectArrayMarkerDisplay() :
  rviz::MarkerDisplay()
{

}

void ObjectArrayMarkerDisplay::onInitialize()
{
  rviz::MarkerDisplay::onInitialize();
}

void ObjectArrayMarkerDisplay::addMarkerArrayMsg(const visualization_msgs::MarkerArray::ConstPtr& array)
{
  ObjectArrayMarkerDisplay::incomingMarkerArray(array);
  float wall_dt, ros_dt;
  update(wall_dt, ros_dt);
}


} // end namespace rviz_plugin_tutorials

// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_plugin_tutorials::MyImuDisplay,rviz::Display )
// PLUGINLIB_EXPORT_CLASS(rviz_plugin_tutorials::ObjectArrayMarkerDisplay,rviz::Display )
// END_TUTORIAL
