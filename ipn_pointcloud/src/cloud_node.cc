/** \file

    This ROS node converts raw Velodyne LIDAR packets to PointCloud2.

*/

#include <ros/ros.h>
#include "ipn_pointcloud/convert.h"

/** Main node entry point. */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "cloud_node");
    ros::NodeHandle node;
    ros::NodeHandle priv_nh("~");

    // create conversion class, which subscribes to raw data
    ipn_pointcloud::Convert conv(node, priv_nh);

    // handle callbacks until shut down
    ros::spin();

    return 0;
}
