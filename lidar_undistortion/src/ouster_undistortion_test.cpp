#include "lidar_undistortion/ouster_undistorter.hpp"
#include "lidar_undistortion/ouster_image_converter.hpp"
#include <pcl/io/pcd_io.h>


using namespace lidar_undistortion;
using OusterPoint = ouster_ros::OS1::PointOS1;
using OusterCloud = pcl::PointCloud<OusterPoint>;

void fillWithDistortedPointcloud(OusterCloud& pc);
void fillWithCorrectedPointCloud(OusterCloud& pc);

int main(int argc, char** argv){
  OusterUndistorter lu(15e9);

  Eigen::Isometry3d pose(Eigen::Isometry3d::Identity());
  pose.matrix()  << 0.517125, -0.829254, -0.211943, 1.11246, 0.786797, 0.558045, -0.263697, 1.21906, 0.336946, -0.0303919, 0.941033, 2.15745, 0, 0, 0, 1;
  lu.addPose(1565309877579768896, pose);
  pose.matrix()  << 0.53849, -0.811225, -0.22791, 1.03782, 0.779227, 0.582343, -0.231692, 1.12807, 0.320676, -0.0528303, 0.945714, 2.16075, 0, 0, 0, 1;
  lu.addPose(1565309877629769564, pose);
  pose.matrix()  << 0.572556, -0.781647, -0.247402, 0.956364, 0.754615, 0.620394, -0.213702, 1.04802, 0.320526, -0.0643372, 0.945052, 2.15529, 0, 0, 0, 1;
  lu.addPose(1565309877679957628, pose);
  pose.matrix()  << 0.622827, -0.731502, -0.277474, 0.865671, 0.706441, 0.678236, -0.202328, 0.989856, 0.336196, -0.0700036, 0.939187, 2.14209, 0, 0, 0, 1;
  lu.addPose(1565309877729769230, pose);
  pose.matrix()  << 0.680385, -0.667836, -0.301781, 0.769285, 0.64855, 0.740449, -0.176404, 0.938014, 0.341262, -0.0756973, 0.936915, 2.13805, 0, 0, 0, 1;
  lu.addPose(1565309877779769182, pose);
  pose.matrix()  << 0.736284, -0.58938, -0.332441, 0.666775, 0.579133, 0.802963, -0.140909, 0.900812, 0.349987, -0.0887783, 0.932538, 2.13994, 0, 0, 0, 1;
  lu.addPose(1565309877829768658, pose);



  OusterCloud oc_input;
  OusterCloud::Ptr oc_output = boost::make_shared<OusterCloud>();

// tedious code that fills of oc_input
  fillWithDistortedPointcloud(oc_input);


  // copy the input before is being modified by processcloud
  pcl::copyPointCloud(oc_input, *oc_output);

  OusterCloud oc_expected;
  OusterImageConverter oic(1024, 64);

  fillWithCorrectedPointCloud(oc_expected);

  // finally process the cloud
  if(!lu.processCloud(oc_output, 1565309877706900736)){
    // if the processing fails, we report failure and quit
    std::cout << "TEST FAILED!" << std::endl;
    return -1;
  }

  size_t counter = 0;
  double error_sum = 0;
  double before_after_error = 0;

  bool cartesian_to_spherical_check = true;

  // compute the error as sum of absolute errors between the 3 coordinates
  for(const auto& point : *oc_output){
    auto point_input = oc_input.points[counter];
    auto point_expected = oc_expected.points[counter++];

    before_after_error = std::abs(point_input.x - point_expected.x) +
                         std::abs(point_input.y - point_expected.y) +
                         std::abs(point_input.z - point_expected.z);

    error_sum = std::abs(point.x - point_expected.x) + std::abs(point.y - point_expected.y) +std::abs(point.z - point_expected.z);

    Eigen::Vector3d spherical = Eigen::Vector3d::Zero();
    Eigen::Vector3d cartesian;
    Eigen::Vector3d cartesian_out;

    cartesian << point_input.x, point_input.y, point_input.z;
    oic.cartesianToSpherical(cartesian, spherical);
    oic.sphericalToCartesian(spherical, cartesian_out);


    cartesian_to_spherical_check &= cartesian.isApprox(cartesian_out, 1e-9);

    if(!cartesian_to_spherical_check){
      // if the processing fails, we report failure and quit
      std::cout << "TEST FAILED! Cartesian to Spherical failed" << std::endl;
      std::cout << cartesian.transpose() << std::endl;
      std::cout << spherical.transpose() << std::endl;
      std::cout << cartesian_out.transpose() << std::endl;
      std::cerr << std::boolalpha << cartesian.isApprox(cartesian_out, 1e-3) << std::endl;

      //return -1;
    }
  }

  // if the sum of all coordinates errors is
  // more thand 5 mm larger from what we expect, we report failure
  if(error_sum < 5e-3){
    std::cout << "TEST PASSED!" << std::endl;
    std::cout << "ERROR is only " << error_sum
              << " while before undistortion it was "
              << before_after_error << "." << std::endl;
    return 0;
  } else {
    std::cout << "Error sum is " << error_sum << " > " << "5e-3" << std::endl;
    std::cout << "TEST FAILED!" << std::endl;
    return -1;
  }
}
