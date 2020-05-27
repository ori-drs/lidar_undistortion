#pragma once
#include "lidar_undistortion/lidar_image_converter.hpp"
#include <ouster_ros/point_os1.h>
#include <ouster/os1_util.h>

namespace lidar_undistortion {
class OusterImageConverter : public LidarImageConverter<ouster_ros::OS1::PointOS1> {
public:
  using OusterPoint = ouster_ros::OS1::PointOS1;
  using OusterCloud = pcl::PointCloud<OusterPoint>;

  OusterImageConverter(int w, int h) : W(w), H(h), pixel_offset_(ouster::OS1::get_px_offset(W)) {

  }

  void convert(const OusterCloud& pc,
               cv::Mat& ranges,
               cv::Mat& altitudes,
               cv::Mat& azimuths) override;

  void rangeImageToMono(const cv::Mat& range_in,
                        cv::Mat& range_out,
                        const double max_range = 30.0) const
  {
    cv::MatConstIterator_<double> img_it;
    cv::MatIterator_<uchar> out_it;
    cv::MatConstIterator_<double> end;
    for(img_it = range_in.begin<double>(),
        end = range_in.end<double>(),
        out_it = range_out.begin<uchar>();
        img_it != end; ++img_it, ++out_it)
    {
      // cap at 30 m and convert into char
      (*out_it) = static_cast<uchar>(255.0 * std::min(*img_it, max_range) / max_range);
    }
  }

private:
  int W;
  int H;
  std::vector<int> pixel_offset_;
};

}



