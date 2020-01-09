/** @file

    This class converts raw Velodyne 3D LIDAR packets to PointCloud2.

*/

#ifndef IPN_POINTCLOUD_CONVERT_H
#define IPN_POINTCLOUD_CONVERT_H

#include <string>

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <ipn_pointcloud/rawdata.h>

namespace ipn_pointcloud
{

class Convert
{
public:
    Convert(ros::NodeHandle node, ros::NodeHandle private_nh, std::string const & node_name = ros::this_node::getName());
    ~Convert() {}

private:
    void processScan(const ipn_msgs::IpnScan::ConstPtr &scanMsg);

    boost::shared_ptr<ipn_pointcloud::RawData> data_;
    ros::Subscriber ipn_scan_;
    ros::Publisher output_;

    boost::shared_ptr<ipn_pointcloud::DataContainerBase> container_ptr_;

    boost::mutex reconfigure_mtx_;
    
    static double max_range;
    static double min_range;
    
};
}  // namespace velodyne_pointcloud

#endif  // IPN_POINTCLOUD_CONVERT_H
