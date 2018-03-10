#include <ros/ros.h>
#include <velodyne_process/heightmap.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "velodyne_process");
    ros::NodeHandle node;
    velodyne_height_map::HeightMap hm(node);
    ros::spin();
    return 0;
}
