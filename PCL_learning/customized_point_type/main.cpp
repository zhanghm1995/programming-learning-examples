#include "point_type_define.h"

int main(int argc, char** argv) {
    pcl::PointCloud<perception::PointXYZIR>::Ptr cloud(new pcl::PointCloud<perception::PointXYZIR>);
    cloud->header.frame_id = "base_link";
    cloud->width = 100;
    cloud->height = 1;
    cloud->points.resize(cloud->width * cloud->height);

    for (size_t i = 0; i < cloud->points.size (); ++i) {
        cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud->points[i].intensity = i;
        cloud->points[i].ring = 100 - i;
    }

    pcl::io::savePCDFile ("test.pcd", *cloud);

    return 0;
}