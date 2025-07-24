#include "lidar_undistortion/ouster_undistorter.hpp"
#include "lidar_undistortion/ouster_image_converter.hpp"
#include "lidar_undistortion/ouster_point.hpp"
#include <gtest/gtest.h>
#include <pcl/io/pcd_io.h>
#include <pwd.h>
#include <filesystem>

using namespace pcl::io;
using namespace lidar_undistortion;
using OusterPoint = PointOuster;
using OusterCloud = pcl::PointCloud<OusterPoint>;

void fillWithDistortedPointcloud(OusterCloud& pc);
void fillWithCorrectedPointCloud(OusterCloud& pc);
void fillWithPropagatedPose(OusterUndistorter& lu);

void getDrsTestingDataPath(std::string& path) {
  const char* homedir;
  const char* drs_data_env = getenv("DRS_TESTING_DATA");
  if (drs_data_env == nullptr) {
    homedir = getenv("HOME");
    if (homedir == nullptr) {
      homedir = getpwuid(getuid())->pw_dir;
    }
    path = std::string(homedir) + "/drs_testing_data";
  } else {
    path = std::string(drs_data_env);
  }
}

TEST(OusterUndistorter, testOS64){
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
  OusterCloud::Ptr oc_output = pcl::make_shared<OusterCloud>();

// tedious code that fills of oc_input
  fillWithDistortedPointcloud(oc_input);


  // copy the input before is being modified by processcloud
  pcl::copyPointCloud(oc_input, *oc_output);

  OusterCloud oc_expected;
  OusterImageConverter oic;

  fillWithCorrectedPointCloud(oc_expected);

  // finally process the cloud
  ASSERT_TRUE(lu.processCloud(oc_output, 1565309877706900736));

  size_t counter = 0;
  double before_after_error = 0;

  // compute the error as sum of absolute errors between the 3 coordinates
  for(const auto& point : *oc_output){
    auto point_input = oc_input.points[counter];
    auto point_expected = oc_expected.points[counter];

    before_after_error = std::abs(point_input.x - point_expected.x) +
                         std::abs(point_input.y - point_expected.y) +
                         std::abs(point_input.z - point_expected.z);

    // EXPECT_NEAR would convert float to double which is not desirable
    ASSERT_LT((point.x - point_expected.x), 1e-5);
    ASSERT_LT((point.y - point_expected.y), 1e-5);
    ASSERT_LT((point.z - point_expected.z), 1e-5);
    Eigen::Vector3d cartesian;
    Eigen::Vector3d spherical;
    Eigen::Vector3d cartesian_out;
    cartesian << point_input.x, point_input.y, point_input.z;
    oic.cartesianToSpherical(cartesian, spherical);
    oic.sphericalToCartesian(spherical, cartesian_out);

    // if the processing fails, we report failure and quit
    EXPECT_TRUE(cartesian.isApprox(cartesian_out, 1e-9))
      << "TEST FAILED! Cartesian to Spherical failed" << std::endl
      << cartesian.transpose() << std::endl
      << spherical.transpose() << std::endl
      << cartesian_out.transpose() << std::endl;
    counter++;
  }
}


TEST(OusterUndistorter, testOS128){
  OusterUndistorter lu(15e9);
  lu.setRingsAndBeams(128,1024);

  // propagated pose runs at 400hz with alphasense imu and ouster runs at 10hz.
  // we need to give about 40 propagated pose.
  fillWithPropagatedPose(lu);

  // Load raw and undistorted point cloud from pcd files
  std::string drs_testing_data;
  getDrsTestingDataPath(drs_testing_data);
  std::filesystem::path drs_testing_data_path(drs_testing_data);

  EXPECT_TRUE(std::filesystem::is_directory(drs_testing_data_path));
  EXPECT_TRUE(drs_testing_data_path.is_complete());

  std::string raw_cloud_file = "lidar_undistortion/raw_1608051903.291503872.pcd";
  auto raw_cloud_path = std::filesystem::canonical(raw_cloud_file, drs_testing_data_path);
  EXPECT_TRUE(std::filesystem::is_regular_file(raw_cloud_path));

  std::string undistorted_pc_file = "lidar_undistortion/undistorted_1608051903.291503872.pcd";
  auto undistorted_cloud_path = std::filesystem::canonical(undistorted_pc_file, drs_testing_data_path);
  EXPECT_TRUE(std::filesystem::is_regular_file(undistorted_cloud_path));

  pcl::PointCloud<PointOuster> raw_point_cloud;
  EXPECT_NE(loadPCDFile(raw_cloud_path.string(), raw_point_cloud), -1);

  pcl::PointCloud<PointOuster> undistorted_point_cloud;
  EXPECT_NE(loadPCDFile(undistorted_cloud_path.string(), undistorted_point_cloud), -1);

  // Undistort raw point cloud
  OusterCloud::Ptr processed_point_cloud = pcl::make_shared<OusterCloud>();
  pcl::copyPointCloud(raw_point_cloud, *processed_point_cloud);
  ASSERT_TRUE(lu.processCloud(processed_point_cloud, 1608051903291503872));

//  pcl::io::savePCDFileASCII<OusterPoint> ("processed.pcd", *processed_point_cloud);

  size_t counter = 0;
  double before_after_error = 0;
  OusterImageConverter oic;

  // compute the error as sum of absolute errors between the 3 coordinates
  // process cloud should be the same as undistorted point cloud
  for(const auto& point : *processed_point_cloud){
    auto point_input = raw_point_cloud.points[counter];
    auto point_expected = undistorted_point_cloud.points[counter];

    ASSERT_LT((point.x - point_expected.x), 1e-5);
    ASSERT_LT((point.y - point_expected.y), 1e-5);
    ASSERT_LT((point.z - point_expected.z), 1e-5);
    Eigen::Vector3d cartesian;
    Eigen::Vector3d spherical;
    Eigen::Vector3d cartesian_out;
    cartesian << point_input.x, point_input.y, point_input.z;
    oic.cartesianToSpherical(cartesian, spherical);
    oic.sphericalToCartesian(spherical, cartesian_out);

    // if the processing fails, we report failure and quit
    EXPECT_TRUE(cartesian.isApprox(cartesian_out, 1e-9))
      << "TEST FAILED! Cartesian to Spherical failed" << std::endl
      << cartesian.transpose() << std::endl
      << spherical.transpose() << std::endl
      << cartesian_out.transpose() << std::endl;
    counter++;
  }

}
