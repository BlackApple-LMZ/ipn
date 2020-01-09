#ifndef _IPN_POINTCLOUD_DATACONTAINERBASE_H
#define _IPN_POINTCLOUD_DATACONTAINERBASE_H

#include <tf/transform_listener.h>
#include <ipn_msgs/IpnScan.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <Eigen/Dense>
#include <string>
#include <algorithm>
#include <cstdarg>

namespace ipn_pointcloud
{
class DataContainerBase
{
public:

    DataContainerBase(const double max_range, const double min_range, const unsigned int init_width, 
                    const unsigned int init_height, const bool is_dense,
                    boost::shared_ptr<tf::TransformListener>& tf_ptr, int fields, ...)
    : config_(max_range, min_range, init_width, init_height, is_dense)
    , tf_ptr(tf_ptr)
    {
        va_list vl; //获取可变参数的列表
        cloud.fields.clear();
        cloud.fields.reserve(fields);
        va_start(vl, fields);
        int offset = 0;
        for (int i = 0; i < fields; ++i)
        {
            // Create the corresponding PointField
            std::string name(va_arg(vl, char*));
            int count(va_arg(vl, int));
            int datatype(va_arg(vl, int));
            offset = addPointField(cloud, name, count, datatype, offset);
        }
        va_end(vl);
        cloud.point_step = offset;
        cloud.row_step = init_width * cloud.point_step;
    }

    struct Config
    {
        double max_range;          ///< maximum range to publish
        double min_range;          ///< minimum range to publish
        unsigned int init_width;
        unsigned int init_height;
        bool is_dense;

        Config(double max_range, double min_range, unsigned int init_width, 
        unsigned int init_height, bool is_dense)
          : max_range(max_range), min_range(min_range), init_width(init_width), init_height(init_height), is_dense(is_dense)
        {
          ROS_INFO_STREAM("Initialized container with "
                          << "min_range: " << min_range << ", max_range: " << max_range
                          << ", init_width: " << init_width << ", init_height: " << init_height << ", is_dense: " << is_dense);
        }
    };

    virtual void setup(const ipn_msgs::IpnScan::ConstPtr& scan_msg)
    {
        cloud.header = scan_msg->header;
        cloud.data.resize(scan_msg->data.size() * cloud.point_step * 0.5);
        cloud.width = config_.init_width;
        cloud.height = config_.init_height;
        cloud.is_dense = static_cast<uint8_t>(config_.is_dense);
    }

    virtual void addPoint(float x, float y, float z, const float distance,
                        const float intensity, const float time) = 0;
    virtual void newLine() = 0;

    const sensor_msgs::PointCloud2& finishCloud()
    {
        cloud.data.resize(cloud.point_step * cloud.width * cloud.height);

        ROS_DEBUG_STREAM("Prepared cloud width" << cloud.height * cloud.width
                                                << " Velodyne points, time: " << cloud.header.stamp);
        return cloud;
    }

    sensor_msgs::PointCloud2 cloud;

    inline void transformPoint(float& x, float& y, float& z)
    {
        Eigen::Vector3f p = transformation * Eigen::Vector3f(x, y, z);
        x = p.x();
        y = p.y();
        z = p.z();
    }

    inline bool pointInRange(float range)
    {
        return (range >= config_.min_range && range <= config_.max_range);
    }

protected:
    Config config_;
    boost::shared_ptr<tf::TransformListener> tf_ptr;  ///< transform listener
    Eigen::Affine3f transformation;
};
} /* namespace velodyne_rawdata */
#endif  // IPN_POINTCLOUD_DATACONTAINERBASE_H
