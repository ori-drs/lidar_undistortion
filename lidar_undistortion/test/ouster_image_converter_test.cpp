#include "lidar_undistortion/ouster_image_converter.hpp"
#include "lidar_undistortion/ouster_point.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <opencv2/highgui.hpp>

using namespace pcl::io;
using namespace lidar_undistortion;

int main(int argc, char** argv){

  pcl::PointCloud<PointOuster> in_cloud;

  loadPCDFile("/home/mcamurri/git/drs_ros_packages/lidar_undistortion/lidar_undistortion/data/new_college.pcd", in_cloud);

  OusterImageConverter oic(1024, 64);

  cv::Mat ranges(64, 1024, CV_64FC1);
  cv::Mat altitudes(64, 1024, CV_64FC1);
  cv::Mat azimuths(64, 1024, CV_64FC1);
  cv::Mat intensities(64, 1024, CV_64FC1);
  cv::Mat reflectivities(64, 1024, CV_64FC1);


  oic.convert(in_cloud, ranges, altitudes, azimuths, intensities, reflectivities);

  cv::Mat ranges_mono(64, 1024, CV_8UC1);

  oic.floatImageToMono(ranges, ranges_mono);

  cv::imwrite("ranges.pgm", ranges_mono);




  return 0;
}

