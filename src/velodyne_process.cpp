#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <velodyne_process/heightmap.h>


ros::Subscriber sub_obstacle;
ros::Publisher pub_cluster;

void obstacleCallback(const sensor_msgs::PointCloud2::ConstPtr inMsg)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_obstacle(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*inMsg, *cloud_obstacle);

    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud(cloud_obstacle);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance(0.7);
    ec.setMinClusterSize(5);
    ec.setMaxClusterSize(500);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_obstacle);
    ec.extract(cluster_indices);

    pcl::PCDWriter writer;

    int j = 0;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZI>);
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        {
            pcl::PointXYZI pt = cloud_obstacle->points[*pit];
            pt.intensity = j;
            cloud_cluster->points.push_back(pt);
        }
        j++;
    }
    cloud_cluster->header.frame_id = "velodyne";
    pub_cluster.publish(cloud_cluster);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "velodyne_process");
    ros::NodeHandle node;
    velodyne_height_map::HeightMap hm(node);

    sub_obstacle = node.subscribe("velodyne_obstacle", 10, &obstacleCallback);
    pub_cluster = node.advertise<pcl::PointCloud<pcl::PointXYZI> >("velodyne_cluster", 10);

    ros::spin();

    return 0;
}
