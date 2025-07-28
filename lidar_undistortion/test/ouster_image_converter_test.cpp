#include "lidar_undistortion/ouster_image_converter.hpp"
#include "lidar_undistortion/ouster_point.hpp"
#include "lidar_undistortion/ouster_metadata_utils.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <opencv2/highgui.hpp>
#include <gtest/gtest.h>
#include <pwd.h> // to get home directory
#include <random>
#include <pcl/range_image/range_image_spherical.h>
#include <filesystem>

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

TEST(OusterImageConverter, DISABLED_convertRangeSpherical){

  std::string drs_testing_data;
  getDrsTestingDataPath(drs_testing_data);
  std::filesystem::path drs_testing_data_path(drs_testing_data);

  EXPECT_TRUE(std::filesystem::is_directory(drs_testing_data_path));
  EXPECT_TRUE(drs_testing_data_path.is_absolute());

  std::string cloud_file = "lidar_undistortion/spot.pcd";
  std::filesystem::path cloud_path(cloud_file);
  auto cloud_path_complete = std::filesystem::canonical(drs_testing_data_path) / cloud_path;
  EXPECT_TRUE(std::filesystem::is_regular_file(cloud_path_complete));

  pcl::PointCloud<PointOuster> in_cloud;
  EXPECT_NE(loadPCDFile(cloud_path_complete.string(), in_cloud), -1);

  EXPECT_TRUE(std::filesystem::is_regular_file(cloud_path_complete));
  EXPECT_NE(loadPCDFile(cloud_path_complete.string(), in_cloud), -1);

  // We now want to create a range image from the above point cloud, with a 1deg angular resolution
   float angularResolution = (float) (  1.0f * (M_PI/180.0f));  //   1.0 degree in radians
   float maxAngleWidth     = (float) (360.0f * (M_PI/180.0f));  // 360.0 degree in radians
   float maxAngleHeight    = (float) (180.0f * (M_PI/180.0f));  // 180.0 degree in radians
   Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
   pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
   float noiseLevel=0.00;
   float minRange = 0.0f;
   int borderSize = 1;

  pcl::RangeImageSpherical spherical;

  spherical.createFromPointCloud(in_cloud, angularResolution, maxAngleWidth, maxAngleHeight,
                                 sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);


  float* ranges = spherical.getRangesArray();




}

TEST(OusterImageConverter, converRangeMono){
  std::string drs_testing_data;
  getDrsTestingDataPath(drs_testing_data);
  std::filesystem::path drs_testing_data_path(drs_testing_data);

  EXPECT_TRUE(std::filesystem::is_directory(drs_testing_data_path));
  EXPECT_TRUE(drs_testing_data_path.is_absolute());

  std::string cloud_file = "lidar_undistortion/spot.pcd";
  std::filesystem::path cloud_path(cloud_file);
  auto cloud_path_complete = std::filesystem::canonical(drs_testing_data_path) / cloud_path;
  EXPECT_TRUE(cloud_path_complete.is_absolute());

  EXPECT_TRUE(std::filesystem::is_regular_file(cloud_path_complete));

  std::string range_file = "lidar_undistortion/spot_ranges.pgm";
  std::filesystem::path range_path(range_file);
  auto range_path_complete = std::filesystem::canonical(drs_testing_data_path) / range_path;
  EXPECT_TRUE(range_path_complete.is_absolute());
  EXPECT_TRUE(std::filesystem::is_regular_file(range_path_complete));

  pcl::PointCloud<PointOuster> in_cloud;

  std::cerr << "Attempt to read " << cloud_path_complete.string() << std::endl;
  EXPECT_NE(loadPCDFile(cloud_path_complete.string(), in_cloud), -1);

  std::string ouster_config_file = "lidar_undistortion/ouster_config.json";
  std::filesystem::path ouster_config_path(ouster_config_file);
  auto ouster_config_path_complete = std::filesystem::canonical(drs_testing_data_path) / ouster_config_path;
  EXPECT_TRUE(ouster_config_path_complete.is_absolute());
  EXPECT_TRUE(std::filesystem::is_regular_file(ouster_config_path_complete));

  std::string metadata_string = read_metadata(ouster_config_path_complete.string());
  auto info  = ouster::sensor::parse_metadata(metadata_string);

  OusterConfig cfg(info);

  OusterImageConverter oic(cfg);

  cv::Mat ranges(64, 1024, CV_64FC1);
  cv::Mat altitudes(64, 1024, CV_64FC1);
  cv::Mat azimuths(64, 1024, CV_64FC1);
  cv::Mat intensities(64, 1024, CV_64FC1);
  cv::Mat reflectivities(64, 1024, CV_64FC1);


  oic.convert(in_cloud, ranges, altitudes, azimuths, intensities, reflectivities);

  cv::Mat ranges_mono(64, 1024, CV_8UC1);

  oic.floatImageToMono(ranges, ranges_mono);

  std::cerr << "Attempt to read " << range_path_complete.string() << std::endl;
  cv::Mat read_range_mono = cv::imread(range_path_complete.string(), cv::ImreadModes::IMREAD_GRAYSCALE);

  cv::Mat diff =  ranges_mono - read_range_mono;

  int nonzero = cv::countNonZero(diff);
  EXPECT_EQ(nonzero, 0);


/*
  cv::namedWindow( "computed", cv::WINDOW_AUTOSIZE );
  cv::imshow("computed", ranges_mono);

  cv::namedWindow( "from_file", cv::WINDOW_AUTOSIZE );
  cv::imshow("from_file", read_range_mono);

  cv::namedWindow( "diff", cv::WINDOW_AUTOSIZE );
  cv::imshow("diff", diff);

  cv::waitKey(0);
*/

}

TEST(OusterImageConverter, cartesianToSpherical){
  OusterImageConverter oic;
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

