#pragma once
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/range_image/range_image_spherical.h>
#include <opencv/cxcore.hpp>

namespace lidar_undistortion {

/**
 * @brief The RangeImageConverter class takes an input
 */

template<class PointT>
class LidarImageConverter {
public:

  virtual void convert(const pcl::PointCloud<PointT>& pc,
                       cv::Mat& ranges,
                       cv::Mat& altitudes,
                       cv::Mat& azimuths,
                       cv::Mat& intensities,
                       cv::Mat& reflectivities) = 0;

  virtual void floatImageToMono(const cv::Mat& img_in,
                                cv::Mat& img_out,
                                const double max_val = 30.0) const
  {
    cv::MatConstIterator_<double> img_it;
    cv::MatIterator_<uchar> out_it;
    cv::MatConstIterator_<double> end;
    for(img_it = img_in.begin<double>(),
        end = img_in.end<double>(),
        out_it = img_out.begin<uchar>();
        img_it != end; ++img_it, ++out_it)
    {
      // for range images, cap at 30 m by default and convert into char
      (*out_it) = static_cast<uchar>(255.0 * std::min(*img_it, max_val) / max_val);
    }
  }
protected:
  /**
   * @brief cartesianToPolar
   * @param cartesian
   * @return
   * @sa cartesianToPolar(const Eigen::Vector3d& cartesian)
   */
   virtual void cartesianToPolar(const Eigen::Vector3d& cartesian,
                    Eigen::Vector3d& polar) {
    polar << cartesian.norm(),
             std::atan2(cartesian(1),cartesian(0)),
             std::acos(cartesian(2) / cartesian.norm());
  }

   /**
    * @brief cartesianToPolar converts a vector in Cartesian coordinates
    * \f$(x, y, z)\f$ into polar coordinates \f$(r, \theta, \phi)\f$, which
    * correspond to range (in mm), beam altitude and azimuth angles (in radians),
    * respectively.
    * @param[in] cartesian vector in cartesian coordinates (in meters)
    * @return a vector representing range, altitude and azimuth angles.
    */
   virtual Eigen::Vector3d cartesianToPolar(const Eigen::Vector3d& cartesian) {
     Eigen::Vector3d polar;
     cartesianToPolar(cartesian, polar);
     return polar;
   }

};


}
