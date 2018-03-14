//
// Created by wxl on 18-2-7.
//

#include <velodyne_process/heightmap.h>
namespace velodyne_height_map
{
    #define MIN(x, y) ((x) < (y) ? (x) : (y))
    #define MAX(x, y) ((x) > (y) ? (x) : (y))

    HeightMap::HeightMap(ros::NodeHandle node)
    {
	    grid_dim_ = 150;
	    m_per_cell_ = 0.2;
	    height_diff_threshold_ = 0.1;
        ROS_INFO_STREAM("height map parameters: "
                                << grid_dim_ << "x" << grid_dim_ << ", "
                                << m_per_cell_ << "m cells, "
                                << height_diff_threshold_ << "m threshold");

        obstacle_publisher_ = node.advertise<VPointCloud>("velodyne_obstacle", 1);
	unknown_publisher_ = node.advertise<VPointCloud>("velodyne_unknown", 1);
        clear_publisher_ = node.advertise<VPointCloud>("velodyne_clear", 1);

        velodyne_scan_ = node.subscribe("velodyne_points", 10, &HeightMap::processData, this, ros::TransportHints().tcpNoDelay(true));
    }

    HeightMap::~HeightMap() {}

    void HeightMap::processData(const VPointCloud::ConstPtr &scan)
    {
        if ((obstacle_publisher_.getNumSubscribers() == 0)
            && (clear_publisher_.getNumSubscribers() == 0)
               && (unknown_publisher_.getNumSubscribers() == 0))
            return;

        // pass along original time stamp and frame ID
        obstacle_cloud_.header.stamp = scan->header.stamp;
        obstacle_cloud_.header.frame_id = scan->header.frame_id;

        // pass along original time stamp and frame ID
        clear_cloud_.header.stamp = scan->header.stamp;
        clear_cloud_.header.frame_id = scan->header.frame_id;

        // pass along original time stamp and frame ID
        unknown_cloud_.header.stamp = scan->header.stamp;
        unknown_cloud_.header.frame_id = scan->header.frame_id;

        // set the exact point cloud size -- the vectors should already have enough space
        size_t npoints = scan->points.size();
        obstacle_cloud_.points.resize(npoints);
        clear_cloud_.points.resize(npoints);
        unknown_cloud_.points.resize(grid_dim_ * grid_dim_);

        size_t obs_count=0;
        size_t empty_count=0;
        size_t unknown_count=0;

        float min[grid_dim_][grid_dim_];
        float max[grid_dim_][grid_dim_];
        unsigned int num[grid_dim_][grid_dim_];
        unsigned char type[grid_dim_][grid_dim_];
        bool init[grid_dim_][grid_dim_];

        for (int x = 0; x < grid_dim_; x++)
        {
            for (int y = 0; y < grid_dim_; y++)
            {
                init[x][y]=false;
                num[x][y] = 0;
                type[x][y] = 0;
            }
        }

        // build height map
        for(unsigned i = 0; i < npoints; ++i)
        {
            int x = ((grid_dim_/2)+scan->points[i].x/m_per_cell_);
            int y = ((grid_dim_/2)+scan->points[i].y/m_per_cell_);

            if(x >= 0 && x < grid_dim_ && y >= 0 && y < grid_dim_)
            {
		        num[x][y] += 1;
                if(!init[x][y])
                {
                    min[x][y] = scan->points[i].z;
                    max[x][y] = scan->points[i].z;
                    init[x][y] = true;
                }
                else
                {
                    min[x][y] = MIN(min[x][y], scan->points[i].z);
                    max[x][y] = MAX(max[x][y], scan->points[i].z);
                }
            }
        }

        // calculate number of obstacles, clear and unknown in each cell
        for(int x = 0; x < grid_dim_; x++)
        {
            for(int y = 0; y < grid_dim_; y++)
            {
                if(num[x][y] >= 1 )
                {

                    if (max[x][y] - min[x][y] > height_diff_threshold_)
                    {
                        type[x][y] = 2;
                    }
                    else
                    {
                        type[x][y] = 1;
                    }

                }
            }
        }

        // create clouds from geid
        double grid_offset = grid_dim_/2*m_per_cell_;
        for (int x = 0; x < grid_dim_; x++)
        {
            for (int y = 0; y < grid_dim_; y++)
            {
                if(type[x][y] == 2)
                {
                    obstacle_cloud_.points[obs_count].x = -grid_offset + (x*m_per_cell_+m_per_cell_/2.0);
                    obstacle_cloud_.points[obs_count].y = -grid_offset + (y*m_per_cell_+m_per_cell_/2.0);
                    obstacle_cloud_.points[obs_count].z = height_diff_threshold_;
                    obs_count ++;
                }

                if (type[x][y] == 1)
                {
                    clear_cloud_.points[empty_count].x = -grid_offset + (x*m_per_cell_+m_per_cell_/2.0);
                    clear_cloud_.points[empty_count].y = -grid_offset + (y*m_per_cell_+m_per_cell_/2.0);
                    clear_cloud_.points[empty_count].z = height_diff_threshold_;
                    empty_count++;
                }

                if (type[x][y] == 0)
                {
                    unknown_cloud_.points[unknown_count].x = -grid_offset + (x*m_per_cell_+m_per_cell_/2.0);
                    unknown_cloud_.points[unknown_count].y = -grid_offset + (y*m_per_cell_+m_per_cell_/2.0);
                    unknown_cloud_.points[unknown_count].z = height_diff_threshold_;
                    unknown_count++;
                }
            }
        }

        obstacle_cloud_.points.resize(obs_count);
        clear_cloud_.points.resize(empty_count);
        unknown_cloud_.points.resize(unknown_count);


        if (obstacle_publisher_.getNumSubscribers() > 0)
            obstacle_publisher_.publish(obstacle_cloud_);
        if (clear_publisher_.getNumSubscribers() > 0)
            clear_publisher_.publish(clear_cloud_);
        if (unknown_publisher_.getNumSubscribers() > 0)
            unknown_publisher_.publish(unknown_cloud_);

    }
}
