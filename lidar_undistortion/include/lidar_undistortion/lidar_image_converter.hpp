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
                       cv::Mat& azimuths) {
//    ri.resize(pc.size());

//    size_t i = 0;
//    Eigen::Vector3d cartesian;
//    Eigen::Vector3d polar;

//    for(auto it = pc.points.begin(); it != pc.points.end(); ++it){
//      cartesian << it->x, it->y, it->z;
//      cartesianToPolar(cartesian, polar);
//      ranges_.at(cv::Point())
//    }


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
    * @param[in] cartesian vector in cartesina coordinates (in meters)
    * @return a vector representing range, altitude and azimuth angles.
    */
   virtual Eigen::Vector3d cartesianToPolar(const Eigen::Vector3d& cartesian) {
     Eigen::Vector3d polar;
     cartesianToPolar(cartesian, polar);
     return polar;
   }

};


}
