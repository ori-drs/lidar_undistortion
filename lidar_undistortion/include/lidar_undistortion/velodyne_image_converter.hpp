#pragma once
#include "lidar_undistortion/lidar_image_converter.hpp"
#include "lidar_undistortion/velodyne_container_organized.hpp"
#include "lidar_undistortion/velodyne_rawdata.hpp"
#include "lidar_undistortion/velodyne_point.hpp"

namespace lidar_undistortion {
class VelodyneImageConverter : public LidarImageConverter<velodyne::PointXYZIRT>
{
public:
  using VelodyneCloud = pcl::PointCloud<velodyne::PointXYZIRT>;

  VelodyneImageConverter() = default;

  VelodyneImageConverter(int w, int h) : W(w), H(h) {

  }

  void scanToPointCloud(const velodyne_msgs::VelodyneScan& scan,
                        VelodyneCloud& cloud)
  {
    // process each packet provided by the driver
    for (size_t i = 0; i < scan.packets.size(); ++i)
    {
      raw_data_.unpack(scan.packets[i], org_cloud_, scan.header.stamp.toNSec());
    }
  }

  void convert(const VelodyneCloud& pc,
               cv::Mat& ranges,
               cv::Mat& altitudes,
               cv::Mat& azimuths) override;

private:
  int W;
  int H;
  velodyne::VelodyneContainerOrganized org_cloud_;
  velodyne::RawData raw_data_;
};

}



