#ifndef IPN_POINTCLOUD_RAWDATA_H
#define IPN_POINTCLOUD_RAWDATA_H

#include <stdint.h>
#include <vector>

#include <ros/ros.h>
#include <ipn_msgs/IpnScan.h>
#include <ipn_pointcloud/datacontainerbase.h>

namespace ipn_pointcloud
{

static const double SCALE = 0.416667; // 1.5*10^8*1000/f/30000 f=12*10^6
static const uint16_t ROTATION_MAX_UNITS = 36000u;  // [deg/100]
static const float ROTATION_RESOLUTION = 0.01f;     // [deg]

/** \brief Velodyne data conversion class */
class RawData
{
public:
    RawData();
    ~RawData()
    {
    }

    void setup();
    void unpack(const ipn_msgs::IpnScan::ConstPtr& pkt, DataContainerBase& data);

private:
    float sin_rot_table_[ROTATION_MAX_UNITS];
    float cos_rot_table_[ROTATION_MAX_UNITS];

};

}  // namespace ipn_pointcloud

#endif  // IPN_POINTCLOUD_RAWDATA_H
