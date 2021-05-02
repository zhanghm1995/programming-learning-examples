/*
 * @Author: Haiming Zhang
 * @Email: zhanghm1995@gmail.com
 * @Date: 2020-05-04 20:56:39
 * @References: 
 * @Description: NA
 */
#ifndef PROJECT_TYPE_DEFINE_H
#define PROJECT_TYPE_DEFINE_H

#include <string>

namespace visualization {

struct ObjectDisplayOptions {
    std::string source_name = "PointRCNN"; // Represent unique objects source
    bool is_display = true; // Whether show this topic
    std::string sub_object_array_topic = "/object_array_lidar_perception_debug";
    std::string pub_vis_marker_topic = "/viz/object_visualizer";
    bool is_show_hollow_bbox = true;
    bool is_show_unknown_objects = false;
    bool is_show_another_color = false;
    bool is_show_polygon = false;
};

} // end visualization
#endif //PROJECT_TYPE_DEFINE_H