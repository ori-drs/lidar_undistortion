#include "lidar_undistortion/velodyne_container_organized.hpp"
#include <boost/make_shared.hpp>

using namespace velodyne;

void VelodyneContainerOrganized::newLine()
{
}

void VelodyneContainerOrganized::setup(const VelodyneContainerConfig& config) {
  config_ = config;
  cloud_indices = std::vector<size_t>(config_.init_height, 0);
  // cloud = boost::make_shared<VelodyneCloud>(config_.init_width, config_.init_height);
  cloud->header.frame_id = "velodyne";
}


void VelodyneContainerOrganized::addPoint(float x,
                                          float y,
                                          float z,
                                          uint16_t ring,
                                          uint16_t azimuth,
                                          float /*distance*/,
                                          float /*intensity*/,
                                          uint64_t time,
                                          const uint16_t /*noise*/,
                                          const uint16_t /*reflectivity*/)
{
  pt.x = x;
  pt.y = y;
  pt.z = z;
  pt.ring = ring;
  pt.time = time;

  // use ring 0 as reference to calculate the pivot azimuth
  if(ring == 0){
    azimuths.push_back(azimuth);
  }
  // std::cerr << "azimuth: " << azimuth << std::endl;
  //std::cerr << azimuths.size() << std::endl;

  cloud->points[ring*cloud->width + cloud_indices[ring]++] = pt;
}
