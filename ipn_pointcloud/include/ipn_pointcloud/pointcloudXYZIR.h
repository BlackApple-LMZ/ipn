#ifndef _IPN_POINTCLOUD_POINTCLOUDXYZIR_H
#define _IPN_POINTCLOUD_POINTCLOUDXYZIR_H

#include <ipn_pointcloud/datacontainerbase.h>
#include <string>

namespace ipn_pointcloud
{
class PointcloudXYZIR : public ipn_pointcloud::DataContainerBase
{
public:
    PointcloudXYZIR(const double max_range, const double min_range, 
                  boost::shared_ptr<tf::TransformListener> tf_ptr = boost::shared_ptr<tf::TransformListener>());

    virtual void newLine();

    virtual void setup(const ipn_msgs::IpnScan::ConstPtr& scan_msg);

    virtual void addPoint(float x, float y, float z, const float distance, const float intensity, const float time);

    sensor_msgs::PointCloud2Iterator<float> iter_x, iter_y, iter_z, iter_intensity, iter_time;
};
}  // namespace ipn_pointcloud

#endif  // IPN_POINTCLOUD_POINTCLOUDXYZIR_H
