#include <ipn_pointcloud/pointcloudXYZIR.h>

namespace ipn_pointcloud 
{

    PointcloudXYZIR::PointcloudXYZIR(
    const double max_range, const double min_range, boost::shared_ptr<tf::TransformListener> tf_ptr)
    : DataContainerBase(
        max_range, min_range, 0, 1, true, tf_ptr, 5,
        "x", 1, sensor_msgs::PointField::FLOAT32,
        "y", 1, sensor_msgs::PointField::FLOAT32,
        "z", 1, sensor_msgs::PointField::FLOAT32,
        "intensity", 1, sensor_msgs::PointField::FLOAT32,
        "time", 1, sensor_msgs::PointField::FLOAT32),
        iter_x(cloud, "x"), iter_y(cloud, "y"), iter_z(cloud, "z"),
        iter_intensity(cloud, "intensity"), iter_time(cloud, "time")
    {};

    void PointcloudXYZIR::setup(const ipn_msgs::IpnScan::ConstPtr& scan_msg){
        DataContainerBase::setup(scan_msg);
        iter_x = sensor_msgs::PointCloud2Iterator<float>(cloud, "x");
        iter_y = sensor_msgs::PointCloud2Iterator<float>(cloud, "y");
        iter_z = sensor_msgs::PointCloud2Iterator<float>(cloud, "z");
        iter_intensity = sensor_msgs::PointCloud2Iterator<float>(cloud, "intensity");
        iter_time = sensor_msgs::PointCloud2Iterator<float >(cloud, "time");
    }

    void PointcloudXYZIR::newLine(){}

    void PointcloudXYZIR::addPoint(float x, float y, float z, float distance, float intensity, float time)
    {
        if(!pointInRange(distance)) return;

        // convert polar coordinates to Euclidean XYZ
        *iter_x = x;
        *iter_y = y;
        *iter_z = z;
        *iter_intensity = intensity;
        *iter_time = time;

        ++cloud.width;
        ++iter_x;
        ++iter_y;
        ++iter_z;
        ++iter_intensity;
        ++iter_time;
    }
}

