#include "lidar_undistortion/velodyne_cloud_organized.hpp"

namespace velodyne_pointcloud
{

  void OrganizedCloudXYZIR::newLine()
  {
    iter_x = iter_x + config_.init_width;
    iter_y = iter_y + config_.init_width;
    iter_z = iter_z + config_.init_width;
    iter_ring = iter_ring + config_.init_width;
    iter_intensity = iter_intensity + config_.init_width;
    ++cloud.height;
  }

  void OrganizedCloudXYZIR::setup(const OrganizedCloudConfig& config){

    config_ = config;
    /*DataContainerBase::setup(scan_msg);
    iter_x = sensor_msgs::PointCloud2Iterator<float>(cloud, "x");
    iter_y = sensor_msgs::PointCloud2Iterator<float>(cloud, "y");
    iter_z = sensor_msgs::PointCloud2Iterator<float>(cloud, "z");
    iter_intensity = sensor_msgs::PointCloud2Iterator<float>(cloud, "intensity");
    iter_ring = sensor_msgs::PointCloud2Iterator<uint16_t >(cloud, "ring");
    iter_time = sensor_msgs::PointCloud2Iterator<float >(cloud, "time");*/
  }


  void OrganizedCloudXYZIR::addPoint(const float& x,
                                     const float& y,
                                     const float& z,
                                     const uint16_t& ring,
                                     const uint16_t& azimuth,
                                     const float& distance,
                                     const float& intensity)
  {
    /** The laser values are not ordered, the organized structure
     * needs ordered neighbour points. The right order is defined
     * by the laser_ring value.
     * To keep the right ordering, the filtered values are set to
     * NaN.
     */
    if (pointInRange(distance))
    {
      *(iter_x+ring) = x;
      *(iter_y+ring) = y;
      *(iter_z+ring) = z;
      *(iter_intensity+ring) = intensity;
      *(iter_ring+ring) = ring;
    }
    else
    {
      *(iter_x+ring) = nanf("");
      *(iter_y+ring) = nanf("");
      *(iter_z+ring) = nanf("");
      *(iter_intensity+ring) = nanf("");
      *(iter_ring+ring) = ring;
    }
  }
}


