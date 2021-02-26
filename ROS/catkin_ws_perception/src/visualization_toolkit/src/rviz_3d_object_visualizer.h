/**
 * @Author: Haiming Zhang
 * @Email: zhanghm1995@gmail.com
 * @Date: 2020-05-04 20:56:39
 * @References: 
 * @Description: Visualize 3D object in Rviz
 */
#ifndef PROJECT_RVIZ_OBJECT_VISUALIZER_H
#define PROJECT_RVIZ_OBJECT_VISUALIZER_H

// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
// Project
#include <iv_object_msgs/ObjectArray.h>
#include "types_define.h"

namespace visualization {

using VObjectArray = iv_object_msgs::ObjectArray;
using VObjectArrayConstPtr = VObjectArray::ConstPtr;

/**
 * @brief To visualize fusion results in Rviz
 */
class Rviz3dObjectVisualizer {
public:
    explicit Rviz3dObjectVisualizer(const ObjectDisplayOptions& options);

    bool Initialize(const ros::Publisher& vis_maker_pub);

    virtual ~Rviz3dObjectVisualizer() = default;

    void ShowObjectAll(const VObjectArrayConstPtr& msg);

//    void ShowObjectCenterPoints(const VObjectArrayConstPtr& msg);

//    void ShowObjectPolygon(const VObjectArrayConstPtr& msg, bool is_show_bbox = true);

    void ShowObjectBBox(const VObjectArrayConstPtr& msg);

    /**
     * @brief Draw the polygon or contour points of objects
     * @param msg
     */
    void ShowObjectPolygon(const VObjectArrayConstPtr& msg);

    void ShowObjectHollowBBox(const VObjectArrayConstPtr& msg);

    void ShowObjectSpeedArrow(const VObjectArrayConstPtr& msg, bool show_ego_abs = true);

    void ShowObjectInfo(const VObjectArrayConstPtr& msg);

private:
    // Publisher
    ros::Publisher vis_marker_pub_;
    ros::Publisher vis_marker_array_pub_;

    ObjectDisplayOptions options_;

    int last_max_markers_num_ = 0;

    int last_max_num_polygon_marker_ = 0;

//    cv::Scalar color_ = cv::Scalar(0, 0, 255); ///< color for show elements, BGR sequence

    std::string source_name_; ///< represent from which sensors or topics, to avoid duplicate overlay
};

}
#endif //PROJECT_RVIZ_OBJECT_VISUALIZER_H