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

  virtual ~LidarImageConverter() = default;

  virtual void convert(const pcl::PointCloud<PointT>& pc,
                       cv::Mat& ranges,
                       cv::Mat& altitudes,
                       cv::Mat& azimuths,
                       cv::Mat& intensities,
                       cv::Mat& reflectivities) = 0;

  virtual void floatImageToMono(const cv::Mat& img_in,
                                cv::Mat& img_out,
                                const double max_val = 30.0,
                                const double min_val = 0) const
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
      (*out_it) = static_cast<uchar>(255.0 * std::min(*img_it - min_val, max_val) / max_val);
    }
  }

  /**
   * @brief cartesianToSpherical converts a vector \f$ \mathbf{v} = (x, y, z)\f$
   * expressed Cartesian coordinates into a vector \f$\mathbf{s} = (r, \theta,
   * \phi)\f$ expressed in spherical coordinates according to the
   * ISO 80000-2:2009 standard
   * (see https://en.wikipedia.org/wiki/Spherical_coordinate_system#Conventions)
   * @param[in] cartesian vector in cartesian coordinates (in meters)
   * @param[out] spherical vector representing range (meters),
   * azimuth (rad) and altitude (rad) angles according to the ISO 80000-2:2009
   * standard
   * @sa cartesianToPolar(const Eigen::Vector3d& cartesian)
   */
   virtual void cartesianToSpherical(const Eigen::Vector3d& cartesian,
                                        Eigen::Vector3d& spherical) const
   {
    double R = cartesian.norm();
    if(R == 0){
      spherical = Eigen::Vector3d::Zero();
      return;
    }
    double phi = std::atan2(cartesian(1),cartesian(0));
    spherical << cartesian.norm(),                           // range
                 std::acos(cartesian(2) / R),                // theta
                 (std::isnan(phi) ? 0 : phi);                // phi
  }

   /**
    * @brief cartesianToPolar converts a vector in Cartesian coordinates
    *  into spherical coordinates
    * @param[in] cartesian
    * @return spherical
    * @sa cartesianToSpherical(const Eigen::Vector3d& cartesian,
                                        Eigen::Vector3d& spherical)
    */
   virtual Eigen::Vector3d cartesianToSpherical(const Eigen::Vector3d& cartesian) const
   {
     Eigen::Vector3d spherical;
     cartesianToSpherical(cartesian, spherical);
     return spherical;
   }

  /**
    * @brief sphericalToCartesian converts a a vector \f$\mathbf{s} = (r,
    * \theta, \phi)\f$ expressed in spherical coordinates according to the
    * ISO 80000-2:2009 standard into a vector\f$ \mathbf{v} = (x, y, z)\f$
    * expressed Cartesian coordinates
    * @param[in] spherical vector representing range (meters),
    * azimuth (rad) and altitude (rad) angles according to the ISO 80000-2:2009
    * standard
    * @param[out] cartesian vector in cartesian coordinates (in meters)
    * @sa cartesianToSpherical(const Eigen::Vector3d& cartesian,
                                     Eigen::Vector3d& spherical)
    */
   virtual void sphericalToCartesian(const Eigen::Vector3d& spherical,
                                           Eigen::Vector3d& cartesian) const
  {
    double sin_theta = std::sin(spherical(1));
    double cos_theta = std::cos(spherical(1));
    double sin_phi = std::sin(spherical(2));
    double cos_phi = std::cos(spherical(2));

    cartesian(0) = spherical(0) * sin_theta * cos_phi;
    cartesian(1) = spherical(0) * sin_theta * sin_phi;
    cartesian(2) = spherical(0) * cos_theta;
  }



};


}
