#include <fstream>
#include <math.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <angles/angles.h>

#include <ipn_pointcloud/rawdata.h>

namespace ipn_pointcloud
{

////////////////////////////////////////////////////////////////////////
//
// RawData base class implementation
//
////////////////////////////////////////////////////////////////////////

RawData::RawData() {}

/** Set up for on-line operation. */
void RawData::setup()
{
    // TODO Set up cached values for sin and cos of all the possible headings set time offsets
    for (uint16_t rot_index = 0; rot_index < ROTATION_MAX_UNITS; ++rot_index) {
        float rotation = angles::from_degrees(ROTATION_RESOLUTION * rot_index);
        cos_rot_table_[rot_index] = cosf(rotation);
        sin_rot_table_[rot_index] = sinf(rotation);
    }
    return ;
}

/** @brief convert raw VLP16 packet to point cloud
*
*  @param pkt raw packet to unpack
*  @param pc shared pointer to point cloud (points are appended)
*/
void RawData::unpack(const ipn_msgs::IpnScan::ConstPtr &pkt, DataContainerBase& data)
{
    float x, y, z;
    float intensity; 
    float distance(0.0);
    
    const uint8_t *raw = &pkt->data[0]; //read shared memory

    for (int i=0; i<pkt->data.size(); i+=2){
        
        distance = raw[i];
        distance += raw[i+1]<<8;
        distance = distance * SCALE;
        
        //TODO calculate the x,y position intensity and time
        x = y = z = distance;
        /** Use standard ROS coordinate system (right-hand rule) */
        float x_coord = y;
        float y_coord = -x;
        float z_coord = z;

        /** Intensity Calculation */
        float min_intensity = 0;
        float max_intensity = 255; 

        //intensity = raw->blocks[block].data[k+2];
        intensity = 0;

        float time = 0;
//        if (timing_offsets.size())
//        time = timing_offsets[block][firing * 16 + dsr];

        data.addPoint(x_coord, y_coord, z_coord, distance, intensity, time);
    }
}
} // namespace velodyne_rawdata
