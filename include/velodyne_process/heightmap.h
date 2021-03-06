#ifndef _HEIGHT_MAP_H_
#define _HEIGHT_MAP_H_

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>

namespace velodyne_height_map
{
    typedef pcl::PointXYZI VPoint;
    typedef pcl::PointCloud<VPoint> VPointCloud;

    class HeightMap
    {
        public:
            HeightMap(ros::NodeHandle node);
            ~HeightMap();
            void processData(const VPointCloud::ConstPtr &scan);
        private:
            int grid_dim_;
            double m_per_cell_;
            double height_diff_threshold_;

            VPointCloud obstacle_cloud_;
            VPointCloud clear_cloud_;
            VPointCloud unknown_cloud_;

            ros::Subscriber velodyne_scan_;
            ros::Publisher obstacle_publisher_;
            ros::Publisher clear_publisher_;
            ros::Publisher unknown_publisher_;
    };
}

#endif
