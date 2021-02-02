
/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2012-2019. All rights reserved.
 * Description: object_display_node.cpp
 * Author: zhanghaiming 00520770
 * Date: Created on 2020-10-12
 * Notes: Display the iv_object_msgs::ObjectArray in Rviz
 */

// C++
#include <vector>
// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <visualization_msgs/MarkerArray.h>
// Yaml
#include <yaml-cpp/yaml.h>
// Projects
#include <iv_object_msgs/ObjectArray.h>
#include "rviz_3d_object_visualizer.h"

using namespace visualization;
using std::cout;
using std::endl;

ObjectDisplayOptions g_display_options;

static bool g_is_show_frame_num = false;

class VisualizerProcess {
public:
    explicit VisualizerProcess(const ros::NodeHandle& nh, const ObjectDisplayOptions& display_options) :
            nh_(nh),
            display_options_(display_options),
            rviz_vis_object_array_(display_options)
    {
        Init();
    }

    bool Init()
    {
        // Define subscribers and publishers
        pub_vis_marker_ = nh_.advertise<visualization_msgs::MarkerArray>(
                display_options_.pub_vis_marker_topic, 20);

        pub_frame_num_marker_ = nh_.advertise<visualization_msgs::MarkerArray>(
                "/viz/frame_num_marker", 20);

        sub_object_array_ = nh_.subscribe<iv_object_msgs::ObjectArray>(
                display_options_.sub_object_array_topic, 20,
                boost::bind(&VisualizerProcess::FusionObjectArrayCallBack, this, _1));

        rviz_vis_object_array_.Initialize(pub_vis_marker_);

        return true;
    }

    void FusionObjectArrayCallBack(const iv_object_msgs::ObjectArray::ConstPtr& msg)
    {
        // TODO: just a temporary solution to show frame number in Rviz, assuming we have
        // tracked objects in every frame
        if (g_is_show_frame_num &&
            display_options_.sub_object_array_topic == "/object_array_lidar_perception_debug") {
            // Get the frame number
            int frame_cnt = 0;
            if (!msg->object_list.empty()) {
                frame_cnt = msg->object_list.begin()->header.seq;
            }

            /// Build the MarkerArray message
            visualization_msgs::MarkerArray info_markers;
            visualization_msgs::Marker marker;
            marker.header.frame_id = "/base_link";
//        marker.header.stamp = ros::Time::now();
            marker.action = visualization_msgs::Marker::ADD;
            marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

            marker.color.r = 1.0;
            marker.color.a = 0.9;

            marker.id = 0;
            marker.ns = "Frame_Num";
            marker.scale.z = 1.0;

            marker.pose.position.x = -2.5;
            marker.pose.position.y = 0.0;
            marker.pose.position.z = 1.0; // height

            // Build the object information
            std::stringstream ss;
            ss <<"#"<< frame_cnt;
            marker.text = ss.str();
            info_markers.markers.push_back(marker);

            // Publish it
            pub_frame_num_marker_.publish(info_markers);
        }

        if (display_options_.is_show_unknown_objects) {
            ROS_WARN("%s======= %.9f #Object: %ld",
                     display_options_.sub_object_array_topic.c_str(),
                     msg->header.stamp.toSec(),
                     msg->object_list.size());
            rviz_vis_object_array_.ShowObjectAll(msg);
        } else {
            iv_object_msgs::ObjectArray::Ptr new_msg(new iv_object_msgs::ObjectArray());
            for (const auto& o : msg->object_list) {
                if (o.type_id == 1 ||
                    o.type_id == 2 ||
                    o.type_id == 3 ||
                    o.type_id == 4 ||
                    o.type_id == 6) {
                    new_msg->object_list.push_back(o);
                }
            }
            new_msg->header = msg->header;

            ROS_WARN("%s======= %.9f #Object: %ld",
                     display_options_.sub_object_array_topic.c_str(),
                     new_msg->header.stamp.toSec(),
                     new_msg->object_list.size());
            rviz_vis_object_array_.ShowObjectAll(new_msg);
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher pub_vis_marker_;
    ros::Publisher pub_frame_num_marker_;
    ros::Subscriber sub_object_array_; //! Subscribe the ObjectArray messages

    ObjectDisplayOptions display_options_;

    Rviz3dObjectVisualizer rviz_vis_object_array_;
};

typedef std::shared_ptr<VisualizerProcess> VisualizerProcessPtr;

namespace YAML {

template<>
struct convert<ObjectDisplayOptions> {

    static bool decode(const Node &node, ObjectDisplayOptions &rhs) {
        rhs.sub_object_array_topic = node["sub_object_array_topic"].as<std::string>();
        rhs.pub_vis_marker_topic = node["pub_vis_marker_topic"].as<std::string>();
        rhs.is_show_hollow_bbox = node["is_show_hollow_bbox"].as<bool>();
        rhs.is_show_unknown_objects = node["is_show_unknown_objects"].as<bool>();
        rhs.is_display = node["is_display"].as<bool>();

        /// Parse some optional parameters
        if (node["is_show_another_color"]) {
            rhs.is_show_another_color = node["is_show_another_color"].as<bool>();
        }
        if (node["is_show_polygon"]) {
            rhs.is_show_polygon = node["is_show_polygon"].as<bool>();
        }
        return true;
    }
};

}

bool ParseConfig(const std::string& config_file, std::vector<ObjectDisplayOptions>& param_vec)
{
    YAML::Node yaml_config = YAML::LoadFile(config_file);

    // Parse global parameters
    g_is_show_frame_num =
            yaml_config["Root"]["is_show_frame_num"].as<bool>();

    // Parse vector parameters
    YAML::Node object_display_config = yaml_config["Object_Display_Node"];

    for (const auto& v : object_display_config) {
        ObjectDisplayOptions parameters = v.as<ObjectDisplayOptions>();
        cout<<parameters.sub_object_array_topic<<" "<<parameters.pub_vis_marker_topic<<endl;
        param_vec.push_back(parameters);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "object_visualizer_node");
    // 1) Parse parameters
    std::string root_path = ros::package::getPath("visualization_toolkit") + "/";
    std::string yaml_config_file = root_path + "config/config_manager.yaml";
    std::vector<ObjectDisplayOptions> display_options_vec;
    ParseConfig(yaml_config_file, display_options_vec);

    // 2) Subscribe topic and callback
    ros::NodeHandle nh, nh_private("~");

    std::vector<VisualizerProcessPtr> visualizer_process_vec;

    for (const auto& option : display_options_vec) {
        if (option.is_display) {
            visualizer_process_vec.emplace_back(new VisualizerProcess(nh, option));
        }
    }

    ros::spin();

    return 0;
}