/**
 * @Author: Haiming Zhang
 * @Email: zhanghm1995@gmail.com
 * @Date: 2020-05-04 20:56:39
 * @References: 
 * @Description: Visualize 3D object in Rviz
 */

#include "rviz_3d_object_visualizer.h"
#include <tf/tf.h>

namespace visualization {

struct ColorRGBA {
    ColorRGBA() = default;
    ColorRGBA(float _r, float _g, float _b, float _a = 1.0) :
        r(_r), g(_g), b(_b), a(_a) {}

    operator std_msgs::ColorRGBA() const
    {
        std_msgs::ColorRGBA ret;
        ret.r = r;
        ret.g = g;
        ret.b = b;
        ret.a = a;
        return ret;
    }

    ColorRGBA& operator=(const ColorRGBA& other)
    {
        this->r = other.r;
        this->g = other.g;
        this->b = other.b;
        this->a = other.a;
    }

    float r = 0.0;
    float g = 0.0;
    float b = 0.0;
    float a = 0.0;
};

static std::map<std::string, ColorRGBA> kColorTable = {
        {std::string("Red"), ColorRGBA(1.0, 0.0, 0.0)},
        {std::string("Green"), ColorRGBA(0.0, 1.0, 0.0)},
        {std::string("SkyBlue"), ColorRGBA(57.0/255, 1.0, 1.0)},
        {std::string("Blue"), ColorRGBA(0.0, 0.0, 1.0)},
        {std::string("Yellow"), ColorRGBA(1.0, 1.0, 0.0)},
        {std::string("Black"), ColorRGBA(0.0, 0.0, 1.0)}};

static std::map<std::string, ColorRGBA> kClassification2Color = {
        {"car", kColorTable["Green"]},
        {"truck", kColorTable["SkyBlue"]},
        {"ped", kColorTable["Blue"]},
        {"cyc", ColorRGBA(0.93, 0.835, 0.45)},
        {"misc", ColorRGBA(1.0, 1.0, 1.0)}
};

/**
## Valid Object classes
#define TARGET_TYPE_OTHER_MOVING 7
#define TARGET_TYPE_OTHER_STATIC 6
#define TARGET_TYPE_BACKGROUND 5
#define TARGET_TYPE_BIGCAR 4
#define TARGET_TYPE_CYCLIST 3
#define TARGET_TYPE_PEDESTRIAN 2
#define TARGET_TYPE_CAR 1
#define TARGET_TYPE_UNKNOWN 0
 */
class ObjectTypeGroup {
public:
    static std::string GetGroupType(uint8_t type_id_id)
    {
        std::string ret = "misc";
        for (const auto& m : s_TypeGroupMap) {
            if (m.second.count(type_id_id)) {
                return m.first;
            }
        }
        return ret;
    }

    static ColorRGBA GetMarkerColor(uint8_t type_id_id, const std::string& use_for)
    {
        ColorRGBA color_ret(1.0, 0.0, 0.0);
        std::string type = GetGroupType(type_id_id);
        if (use_for == "BBox") {
            color_ret = kClassification2Color[type];
        } else if (use_for == "Speed") {
            color_ret = ColorRGBA(0.0, 1.0, 1.0, 0.9);
//            color_ret = ColorRGBA(1.0, 0.0, 0.0, 0.9);
        } else if (use_for == "Text") {
            color_ret = ColorRGBA(0.0, 1.0, 1.0, 1.0);
        } else {
            // Do nothing
        }

        return color_ret;
    }

private:
    static std::map<std::string, std::set<int> > s_TypeGroupMap;
};

std::map<std::string, std::set<int> > ObjectTypeGroup::s_TypeGroupMap = {
        {"car", {1}},
        {"truck", {4}},
        {"ped", {2}},
        {"cyc", {3}}
};

template <typename T, typename... Args>
std::unique_ptr<T> make_unique(Args&&... args) {
    return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}

class ObjectColorManager {
public:
    enum class Scheme { Default, Detailed };

    ObjectColorManager() = default;

    std::string GetGroupType(uint8_t type_id_id)
    {
        std::string ret = "misc";
        for (const auto& m : TypeGroupMap_) {
            if (m.second.count(type_id_id)) {
                return m.first;
            }
        }
        return ret;
    }

    ColorRGBA GetMarker(const uint8_t type_id_id, const std::string& use_for)
    {
        ColorRGBA color_ret(1.0, 0.0, 0.0);
        std::string type = GetGroupType(type_id_id);
        if (use_for == "BBox") {
            color_ret = Classification2Color_[type];
        } else if (use_for == "Speed") {
            color_ret = ColorRGBA(0.0, 1.0, 1.0, 0.9);
//            color_ret = ColorRGBA(1.0, 0.0, 0.0, 0.9);
        } else if (use_for == "Text") {
            color_ret = ColorRGBA(0.0, 1.0, 1.0, 1.0);
        } else {
            // Do nothing
        }

        return color_ret;
    }

    void SetType2ColorMap(const std::map<std::string, ColorRGBA>& color_map)
    {
        Classification2Color_ = color_map;
    }

    static std::unique_ptr<ObjectColorManager> Instance(
            const ObjectColorManager::Scheme& color_scheme = ObjectColorManager::Scheme::Default)
    {
        auto color_manager = ObjectColorManager();

        if (color_scheme == ObjectColorManager::Scheme::Default) {
            return make_unique<ObjectColorManager>(color_manager);
        } else if (color_scheme == ObjectColorManager::Scheme::Detailed) {
            std::map<std::string, ColorRGBA> type2color_map = {
                    {"car", kColorTable["Green"]},
                    {"truck", kColorTable["SkyBlue"]},
                    {"ped", kColorTable["Blue"]},
                    {"cyc", ColorRGBA(0.93, 0.835, 0.45)},
                    {"misc", ColorRGBA(1.0, 1.0, 1.0)}
            };
            color_manager.SetType2ColorMap(type2color_map);
            return make_unique<ObjectColorManager>(color_manager);
        }
    }

private:
    std::map<std::string, ColorRGBA> Classification2Color_ = {
            {"car", kColorTable["Green"]},
            {"truck", kColorTable["Green"]},
            {"ped", kColorTable["Blue"]},
            {"cyc", ColorRGBA(0.93, 0.835, 0.45)},
//        {"cyc", kColorTable["Blue"]},
            {"misc", ColorRGBA(1.0, 1.0, 1.0)}
    };

    std::map<std::string, std::set<int> > TypeGroupMap_ = {
            {"car", {1, 6}},
            {"truck", {4}},
            {"ped", {2}},
            {"cyc", {3}}
    };
};

struct Point3f {
    float x, y, z;
    Point3f(float x, float y, float z)
            :x(x), y(y), z(z) {}
};

void GetPointsFromBox(std::vector<Point3f>& points,
                      double Orient, double L, double W, double centerX, double centerY)
{
    float px, py;
    int signs[4][2] = {{1, 1}, {-1, 1}, {-1, -1}, {1, -1}};
    for (int i = 0; i < 4; ++i) {
        double tempx = 1 / 2.0f * (signs[i][0] * L * std::cos(Orient) - signs[i][1] * W * std::sin(Orient));
        double tempy = 1 / 2.0f * (signs[i][0] * L * std::sin(Orient) + signs[i][1] * W * std::cos(Orient));
        px = centerX + tempx;
        py = centerY + tempy;
        points.emplace_back(Point3f(px, py, 0.0));
    }
}

////////////////////////////////////Rviz3dObjectVisualizer//////////////////////////////////////////////////
Rviz3dObjectVisualizer::Rviz3dObjectVisualizer(const ObjectDisplayOptions& options) : options_(options)
{
    source_name_ = options.source_name;
}

bool Rviz3dObjectVisualizer::Initialize(const ros::Publisher& vis_maker_pub)
{
    vis_marker_array_pub_ = vis_maker_pub;
    return true;
}

void Rviz3dObjectVisualizer::ShowObjectAll(const VObjectArrayConstPtr& msg)
{
    if (options_.is_show_hollow_bbox) {
        ShowObjectHollowBBox(msg);
    } else {
        // Show bboxes
        ShowObjectBBox(msg);
    }

    if (options_.is_show_polygon) {
        ShowObjectPolygon(msg);
    }

    // Show speed;
    ShowObjectSpeedArrow(msg);

    // Show info
    ShowObjectInfo(msg);
}

void Rviz3dObjectVisualizer::ShowObjectHollowBBox(const VObjectArrayConstPtr& msg)
{
    visualization_msgs::MarkerArray bbox_markers;

    for (std::size_t i = 0; i < msg->object_list.size(); ++i) {
        const auto& obj = msg->object_list[i];
        double x = msg->object_list[i].center.x;
        double y = msg->object_list[i].center.y;
        double z = msg->object_list[i].center.z;
        double orient = obj.orientation;
        double sx = obj.size.x;
        double sy = obj.size.y;
        double sz = obj.size.z; // height

        // for box------------------------------------------------------------------------------
        visualization_msgs::Marker marker;
        marker.header.frame_id = "/base_link";
//        marker.header.stamp = ros::Time::now();
        marker.action = visualization_msgs::Marker::ADD;

        ColorRGBA color;
        if (!options_.is_show_another_color) {
            color = ObjectTypeGroup::GetMarkerColor(obj.type_id, "BBox");
        } else {
            color = ColorRGBA(1.0, 0.0, 1.0);
        }
        marker.color = color;

        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.id = static_cast<int>(i);
        marker.ns = source_name_ + "ObjectHBBox";
        marker.scale.x = 0.1;

        std::vector<Point3f> points;
        GetPointsFromBox(points, orient, sx, sy, x, y);

        auto no_poly = points.size();
        auto pointNum = 2 * no_poly;
        std::vector<geometry_msgs::Point> cubPoints(pointNum);
        for (int i = 0; i < points.size(); i++) {
            cubPoints[i].x = points[i].x;
            cubPoints[i].y = points[i].y;
            cubPoints[i].z = z - sz / 2.0;
            cubPoints[i + no_poly].x = points[i].x;
            cubPoints[i + no_poly].y = points[i].y;
            cubPoints[i + no_poly].z = z + sz / 2.0;
        }

        marker.points.clear();

        for (int i = 0; i < no_poly; i++) {
            int start_idx=i;
            marker.points.push_back(cubPoints[start_idx]);
            int end_idx=i+1;
            if(end_idx>=no_poly) end_idx-=no_poly;
            marker.points.push_back(cubPoints[end_idx]);
        }
        for (int i = 0;i < no_poly;i++) {
            int start_idx=i;
            marker.points.push_back(cubPoints[start_idx+no_poly]);
            int end_idx=i+1;
            if(end_idx>=no_poly) end_idx-=no_poly;
            marker.points.push_back(cubPoints[end_idx+no_poly]);
        }
        for(int i = 0; i < no_poly; i++) {
            int start_idx=i;
            marker.points.push_back(cubPoints[start_idx]);
            int end_idx = start_idx + no_poly;
            marker.points.push_back(cubPoints[end_idx]);
        }

        bbox_markers.markers.push_back(marker);
    }
    vis_marker_array_pub_.publish(bbox_markers);

    // ---------------remove markers not update--------------------
    int object_num = (int)msg->object_list.size();
    last_max_markers_num_ = last_max_markers_num_ < object_num ? object_num : last_max_markers_num_;

    visualization_msgs::MarkerArray polygon_markers_remove;
    for (int i = object_num; i < last_max_markers_num_; ++i) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "/base_link";
//        marker.header.stamp = ros::Time::now();
        marker.action = visualization_msgs::Marker::DELETE;
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.id = i;
        marker.ns = source_name_ + "ObjectHBBox";
        polygon_markers_remove.markers.push_back(marker);
    }

    vis_marker_array_pub_.publish(polygon_markers_remove);
}

void Rviz3dObjectVisualizer::ShowObjectBBox(const VObjectArrayConstPtr& msg)
{
    visualization_msgs::MarkerArray bbox_markers;

    for (std::size_t i = 0; i < msg->object_list.size(); ++i) {
        const auto& obj = msg->object_list[i];
        double x = msg->object_list[i].center.x;
        double y = msg->object_list[i].center.y;
        double z = msg->object_list[i].center.z;
        double orient = obj.orientation;
        double sx = obj.size.x;
        double sy = obj.size.y;
        double sz = obj.size.z; // height

        // for box------------------------------------------------------------------------------
        visualization_msgs::Marker marker;
        marker.header.frame_id = "/base_link";
//        marker.header.stamp = ros::Time::now();
        marker.action = visualization_msgs::Marker::ADD;

        ColorRGBA color;
        if (!options_.is_show_another_color) {
            color = ObjectTypeGroup::GetMarkerColor(obj.type_id, "BBox");
        } else {
            color = ColorRGBA(1.0, 0.0, 1.0);
        }
        marker.color = color;
        marker.color.a = 0.7;

        marker.type = visualization_msgs::Marker::CUBE;
        marker.id = static_cast<int>(i);
        marker.ns = source_name_ + "ObjectBBox";
        marker.scale.x = sx;
        marker.scale.y = sy;
        marker.scale.z = sz;

        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = z;

        tf::Quaternion q;
        q.setEuler(0.0, 0.0, orient);
        marker.pose.orientation.x = q.x();
        marker.pose.orientation.y = q.y();
        marker.pose.orientation.z = q.z();
        marker.pose.orientation.w = q.w();
        bbox_markers.markers.push_back(marker);
    }
    vis_marker_array_pub_.publish(bbox_markers);

    // ---------------remove markers not update--------------------
    int object_num = (int)msg->object_list.size();
    last_max_markers_num_ = last_max_markers_num_ < object_num ? object_num : last_max_markers_num_;

    visualization_msgs::MarkerArray polygon_markers_remove;
    for (int i = object_num; i < last_max_markers_num_; ++i) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "/base_link";
//        marker.header.stamp = ros::Time::now();
        marker.action = visualization_msgs::Marker::DELETE;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.id = i;
        marker.ns = source_name_ + "ObjectBBox";
        polygon_markers_remove.markers.push_back(marker);
    }

    vis_marker_array_pub_.publish(polygon_markers_remove);
}

void Rviz3dObjectVisualizer::ShowObjectPolygon(const VObjectArrayConstPtr& msg)
{
    visualization_msgs::MarkerArray polygon_markers;

    int num_polygon_markers = 0;
    for (std::size_t i = 0; i < msg->object_list.size(); ++i) {
        const auto& obj = msg->object_list[i];
        const auto x = obj.center.x;
        const auto y = obj.center.y;
        const auto z = obj.center.z;
        const auto sx = obj.size.x;
        const auto sy = obj.size.y;
        const auto sz = obj.size.z; // height

        const auto& contour_pts_vec = obj.polygon.points;

        if (contour_pts_vec.size() < 3) {
            continue;
        }

        // for polygon-------------------------------------------------
        visualization_msgs::Marker marker;
        marker.header.frame_id = "/base_link";
//        marker.header.stamp = ros::Time::now();
        marker.action = visualization_msgs::Marker::ADD;

        ColorRGBA color;
        if (!options_.is_show_another_color) {
            color = ObjectTypeGroup::GetMarkerColor(obj.type_id, "BBox");
        } else {
            color = ColorRGBA(1.0, 0.0, 1.0);
        }
//        marker.color = color;
        marker.color.r = 1.0;
        marker.color.a = 0.9;

        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.id = num_polygon_markers;
        marker.ns = source_name_ + "ObjectPolygon";
        marker.scale.x = 0.1;

        for (const auto& contour_pt : contour_pts_vec) {
            geometry_msgs::Point pt;
            pt.x = contour_pt.x;
            pt.y = contour_pt.y;
            pt.z = z;
            marker.points.push_back(pt);
        }

        /// Add the first point to construct a closed polygon
        const auto contour_first_pt = *contour_pts_vec.begin();
        geometry_msgs::Point pt_first;
        pt_first.x = contour_first_pt.x;
        pt_first.y = contour_first_pt.y;
        pt_first.z = z;
        marker.points.push_back(pt_first);

        polygon_markers.markers.push_back(marker);
        ++num_polygon_markers;
    }
    vis_marker_array_pub_.publish(polygon_markers);

    // ---------------remove markers not update--------------------
    last_max_num_polygon_marker_ = last_max_num_polygon_marker_ < num_polygon_markers ?
                                   num_polygon_markers : last_max_num_polygon_marker_;

    visualization_msgs::MarkerArray polygon_markers_remove;
    for (int i = num_polygon_markers; i < last_max_num_polygon_marker_; ++i) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "/base_link";
//        marker.header.stamp = ros::Time::now();
        marker.action = visualization_msgs::Marker::DELETE;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.id = i;
        marker.ns = source_name_ + "ObjectPolygon";
        polygon_markers_remove.markers.push_back(marker);
    }

    vis_marker_array_pub_.publish(polygon_markers_remove);
}

void Rviz3dObjectVisualizer::ShowObjectSpeedArrow(
        const VObjectArrayConstPtr& msg, bool show_ego_abs)
{
    visualization_msgs::MarkerArray arrow_markers;
    for (std::size_t i = 0; i < msg->object_list.size(); ++i) {
        const auto& obj = msg->object_list[i];
        double x = msg->object_list[i].center.x;
        double y = msg->object_list[i].center.y;
        const double z = msg->object_list[i].center.z;
        double vx = msg->object_list[i].velocity.x;
        double vy = msg->object_list[i].velocity.y;
        double sz = obj.size.z; // height
        double vx_abs = msg->object_list[i].absolute_velocity.x;
        double vy_abs = msg->object_list[i].absolute_velocity.y;

        visualization_msgs::Marker marker;
        double s = sqrt(vx * vx + vy * vy);
        double h = atan2(vy, vx);
        if (show_ego_abs) {
            s = std::hypot(vx_abs, vy_abs);
            h = std::atan2(vy_abs, vx_abs);
        }

        marker.header.frame_id = "/base_link";
//        marker.header.stamp = ros::Time::now();
        marker.action = visualization_msgs::Marker::ADD;
        auto color = ObjectTypeGroup::GetMarkerColor(obj.type_id, "Speed");
        marker.color = color;

        marker.type = visualization_msgs::Marker::ARROW;
        marker.id = static_cast<int>(3 * i) + 1;
        marker.ns = source_name_ + "ObjectSpeed";
        marker.scale.x = 0.5 * s;
        marker.scale.y = 0.25;
        marker.scale.z = 0.2;

        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z =  z + sz / 2; // height
        tf::Quaternion q;
        q.setEuler(0, 0, h);
        marker.pose.orientation.x = q.x();
        marker.pose.orientation.y = q.y();
        marker.pose.orientation.z = q.z();
        marker.pose.orientation.w = q.w();
        arrow_markers.markers.push_back(marker);
    }

    // -------------delete not update markers------------------
    int object_num = (int)msg->object_list.size();
    last_max_markers_num_ = last_max_markers_num_ < object_num ? object_num : last_max_markers_num_;

    for (int i = object_num; i < last_max_markers_num_; ++i) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "/base_link";
//        marker.header.stamp = ros::Time::now();
        marker.action = visualization_msgs::Marker::DELETE;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.id = 3 * i + 1;
        marker.ns = source_name_ + "ObjectSpeed";
        arrow_markers.markers.push_back(marker);
    }

    vis_marker_array_pub_.publish(arrow_markers);
}

void Rviz3dObjectVisualizer::ShowObjectInfo(const VObjectArrayConstPtr& msg)
{
    visualization_msgs::MarkerArray info_markers;
    for (std::size_t i = 0; i < msg->object_list.size(); ++i) {
        const auto& obj = msg->object_list[i];
        double x = msg->object_list[i].center.x;
        double y = msg->object_list[i].center.y;
        double height = msg->object_list[i].size.z;

        // Build marker
        visualization_msgs::Marker marker;
        marker.header.frame_id = "/base_link";
//        marker.header.stamp = ros::Time::now();
        marker.action = visualization_msgs::Marker::ADD;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

        auto color = ObjectTypeGroup::GetMarkerColor(obj.type_id, "Text");
        marker.color = color;
        marker.color.a = 0.9;

        marker.id = static_cast<int>(i);
        marker.ns = source_name_ + "ObjectInfo";
        marker.scale.z = 1.0;

        marker.pose.position.x = x;
        marker.pose.position.y = y - 2;
        marker.pose.position.z = height + 1.0; // height

        // Build the object information
        std::stringstream ss;
        ss <<"id="<< msg->object_list[i].id;

        marker.text = ss.str();

        info_markers.markers.push_back(marker);
    }

    // -------------delete not update markers------------------
    int object_num = (int)msg->object_list.size();
    last_max_markers_num_ = last_max_markers_num_ < object_num ? object_num : last_max_markers_num_;

    for (int i = object_num; i < last_max_markers_num_; ++i) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "/base_link";
//        marker.header.stamp = ros::Time::now();
        marker.action = visualization_msgs::Marker::DELETE;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.id = i;
        marker.ns = source_name_ + "ObjectInfo";
        info_markers.markers.push_back(marker);
    }

    vis_marker_array_pub_.publish(info_markers);
}

} // namespace visualization