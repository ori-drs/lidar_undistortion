#include "lidar_undistortion/ouster_image_converter.hpp"
#include "lidar_undistortion/ouster_point.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <opencv2/highgui.hpp>
#include <gtest/gtest.h>
#include <pwd.h> // to get home directory
#include <random>

using namespace pcl::io;
using namespace lidar_undistortion;

void getDrsTestingDataPath(std::string &path) {
  const char *homedir;
  const char *drs_data_env = getenv("DRS_TESTING_DATA");
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

TEST(OusterImageConverter, converRangeMono){
  std::string drs_testing_data;
  getDrsTestingDataPath(drs_testing_data);
  boost::filesystem::path drs_testing_data_path(drs_testing_data);

  EXPECT_TRUE(boost::filesystem::is_directory(drs_testing_data_path));
  EXPECT_TRUE(drs_testing_data_path.is_complete());

  std::string cloud_file = "lidar_undistortion/new_college.pcd";
  boost::filesystem::path cloud_path(cloud_file);
  auto cloud_path_complete = boost::filesystem::canonical(cloud_path, drs_testing_data_path);
  EXPECT_TRUE(cloud_path_complete.is_complete());

  std::string range_file = "lidar_undistortion/ranges.pgm";
  boost::filesystem::path range_path(range_file);
  auto range_path_complete = boost::filesystem::canonical(range_path, drs_testing_data_path);
  EXPECT_TRUE(range_path_complete.is_complete());

  pcl::PointCloud<PointOuster> in_cloud;

  EXPECT_NE(loadPCDFile(cloud_path_complete.string(), in_cloud), -1);

  OusterImageConverter oic(1024, 64);

  cv::Mat ranges(64, 1024, CV_64FC1);
  cv::Mat altitudes(64, 1024, CV_64FC1);
  cv::Mat azimuths(64, 1024, CV_64FC1);
  cv::Mat intensities(64, 1024, CV_64FC1);
  cv::Mat reflectivities(64, 1024, CV_64FC1);


  oic.convert(in_cloud, ranges, altitudes, azimuths, intensities, reflectivities);

  cv::Mat ranges_mono(64, 1024, CV_8UC1);

  oic.floatImageToMono(ranges, ranges_mono);

  cv::Mat read_range_mono = cv::imread(range_path_complete.string(), cv::ImreadModes::IMREAD_GRAYSCALE);

  cv::Mat diff =  ranges_mono - read_range_mono;

  int nonzero = cv::countNonZero(diff);
  EXPECT_EQ(nonzero, 0);


/*
  cv::namedWindow( "first", cv::WINDOW_AUTOSIZE );
  cv::imshow("first", ranges_mono);

  cv::namedWindow( "second", cv::WINDOW_AUTOSIZE );
  cv::imshow("second", read_range_mono);

  cv::namedWindow( "diff", cv::WINDOW_AUTOSIZE );
  cv::imshow("diff", diff);

  cv::waitKey(0);
*/

}

TEST(OusterImageConverter, cartesianToSpherical){
  OusterImageConverter oic(1024, 64);
  std::random_device rd;
  std::mt19937_64 gen(rd());

  // randomly generate points in within a cube of 100 x 100 x 100 meters
  std::uniform_real_distribution<> disr(0, 100);

  for(int i = 0; i < 100; i++){
    double x = disr(gen);
    double y = disr(gen);
    double z = disr(gen);

    Eigen::Vector3d cartesian;
    Eigen::Vector3d spherical;
    Eigen::Vector3d cartesian_out;
    cartesian << x, y, z;
    oic.cartesianToSpherical(cartesian, spherical);
    oic.sphericalToCartesian(spherical, cartesian_out);
    EXPECT_NEAR(cartesian(0), cartesian_out(0), 1e-5);
    EXPECT_NEAR(cartesian(1), cartesian_out(1), 1e-5);
    EXPECT_NEAR(cartesian(2), cartesian_out(2), 1e-5);
  }
}

