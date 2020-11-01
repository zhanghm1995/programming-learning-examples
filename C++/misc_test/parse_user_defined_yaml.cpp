/*======================================================================
* Author   : Haiming Zhang
* Email    : zhanghm_1995@qq.com
* Version  :　2018年7月6日
* Copyright    :
* Descriptoin : Learn how to use yaml-cpp library to parse your customized yaml type
* References  : https://github.com/jbeder/yaml-cpp/wiki/Tutorial
======================================================================*/

// C++
#include <vector>
#include <iostream>
// Yaml
#include <yaml-cpp/yaml.h>

using std::cout;
using std::endl;

struct ObjectDisplayOptions {
    std::string source_name = "KittiTrack"; // Represent unique objects source
    bool is_display = true; // Whether show this topic
    std::string sub_object_array_topic = "/kitti_object";
    std::string pub_vis_marker_topic = "/viz/object_visualizer";
    bool is_show_hollow_bbox = true;
};

class VisualizerProcess {
public:
    explicit VisualizerProcess(const ObjectDisplayOptions& display_options) :
            display_options_(display_options)
    {
        Init();
    }

    bool Init()
    {
        std::cout<<"This is a VisualizerProcess class use "<<display_options_.source_name<<" display options!"<<std::endl;
        return true;
    }
private:
    ObjectDisplayOptions display_options_;
};

typedef std::shared_ptr<VisualizerProcess> VisualizerProcessPtr;

namespace YAML {

template<>
struct convert<ObjectDisplayOptions> {

    static bool decode(const Node &node, ObjectDisplayOptions &rhs) {
        rhs.sub_object_array_topic = node["sub_object_array_topic"].as<std::string>();
        rhs.pub_vis_marker_topic = node["pub_vis_marker_topic"].as<std::string>();
        rhs.is_show_hollow_bbox = node["is_show_hollow_bbox"].as<bool>();
        rhs.is_display = node["is_display"].as<bool>();
        return true;
    }
};

}

bool ParseConfig(const std::string& config_file, std::vector<ObjectDisplayOptions>& param_vec)
{
    YAML::Node yaml_config = YAML::LoadFile(config_file);
    YAML::Node object_display_config = yaml_config["Object_Display_Node"];

    for (const auto& v : object_display_config) {
        ObjectDisplayOptions parameters = v.as<ObjectDisplayOptions>();
        cout<<parameters.sub_object_array_topic<<" "<<parameters.pub_vis_marker_topic<<endl;
        param_vec.push_back(parameters);
    }
}

int main(int argc, char** argv)
{
    // 1) Parse parameters
    std::string yaml_config_file =  "../config/config_manager.yaml";
    std::vector<ObjectDisplayOptions> display_options_vec;
    ParseConfig(yaml_config_file, display_options_vec);

    // 2) Subscribe topic and callback
    std::vector<VisualizerProcessPtr> visualizer_process_vec;

    for (const auto& option : display_options_vec) {
        if (option.is_display) {
            visualizer_process_vec.emplace_back(new VisualizerProcess(option));
        }
    }
    return 0;
}