/*
 * @Author: Haiming Zhang
 * @Email: zhanghm_1995@qq.com
 * @Date: 2020-04-23 21:52:15
 * @References: 
 * @Description: Learn the occupancy grid map represatation in RViz
 */

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "occupancy_grid_map");
    ros::NodeHandle nh;

    ros::Publisher pub_occupancy_grid = nh.advertise<nav_msgs::OccupancyGrid>(
        "occupancy_map", 10, true);

    nav_msgs::OccupancyGrid map;
    map.header.frame_id = "base_link";
    map.header.stamp = ros::Time::now();
    map.info.width = 20;
    map.info.height = 20;
    map.info.resolution = 1; // 0.2
    map.info.origin.position.x = 10;
    map.info.origin.position.y = 5;

    int cell_num = map.info.width * map.info.height;
    int p[cell_num] = {0};
    p[10] = 100;
    p[25] = 50;
    
    std::vector<int8_t> vec(p, p + cell_num);
    map.data = vec;

    while (ros::ok()) {
        pub_occupancy_grid.publish(map);
    }

    ros::shutdown();

    return 0;
}