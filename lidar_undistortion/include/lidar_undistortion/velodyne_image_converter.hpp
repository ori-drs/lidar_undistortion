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

    velodyne::RawDataConfig cfg;

    cfg.calibrationFile = "/home/mcamurri/git/anymal_research/anymal/anymal/anymal_drivers/anymal_velodyne/calib/VLP16db_example.yaml";
    cfg.model = "VLP16";
    cfg.min_range = 0;
    cfg.max_range = 30;
    cfg.view_direction = 0;
    cfg.view_width = 2*M_PI;
    raw_data_.setup(cfg);
  }

  void scanToPointCloud(const velodyne_msgs::VelodyneScan& scan,
                        VelodyneCloud& cloud)
  {
    // process each packet provided by the driver
    for (size_t i = 0; i < scan.packets.size(); ++i) {
      raw_data_.unpack(scan.packets[i], org_cloud_, scan.header.stamp.toNSec());
    }

    auto cloud_ptr = org_cloud_.getCloud();
    cloud = *cloud_ptr;
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



