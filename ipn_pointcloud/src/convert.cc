/** @file

    This class converts raw Velodyne 3D LIDAR packets to PointCloud2.

*/

#include "ipn_pointcloud/convert.h"
#include <ipn_pointcloud/pointcloudXYZIR.h>

namespace ipn_pointcloud
{
  
double Convert::max_range = 12.5;
double Convert::min_range = 0.8;

  /** @brief Constructor. */
Convert::Convert(ros::NodeHandle node, ros::NodeHandle private_nh, std::string const & node_name):
    data_(new ipn_pointcloud::RawData())
{
    data_->setup();

    //initialize ros pointcloud2
    container_ptr_ = boost::shared_ptr<PointcloudXYZIR>(new PointcloudXYZIR(max_range, min_range));

    // advertise output point cloud (before subscribing to input data)
    output_ = node.advertise<sensor_msgs::PointCloud2>("ipn_points", 10);

    // subscribe to VelodyneScan packets
    ipn_scan_ = node.subscribe("ipn_packets", 10, &Convert::processScan, (Convert *) this, ros::TransportHints().tcpNoDelay(true));

}

  /** @brief Callback for raw scan messages. */
void Convert::processScan(const ipn_msgs::IpnScan::ConstPtr &scanMsg)
{
    if (output_.getNumSubscribers() == 0)         // no one listening?
        return;                                     // avoid much work

    boost::lock_guard<boost::mutex> guard(reconfigure_mtx_);
    
    // allocate a point cloud with same time and frame ID as raw data
    container_ptr_->setup(scanMsg);
    data_->unpack(scanMsg, *container_ptr_);

    // publish the accumulated cloud message
    output_.publish(container_ptr_->finishCloud());
}

} // namespace velodyne_pointcloud
