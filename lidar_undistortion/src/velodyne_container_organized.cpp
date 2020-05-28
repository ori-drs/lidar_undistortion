#include "lidar_undistortion/velodyne_container_organized.hpp"

using namespace velodyne;

void VelodyneContainerOrganized::newLine()
{
  *iter_x =         *iter_x + config_.init_width;
  *iter_y =         *iter_y + config_.init_width;
  *iter_z =         *iter_z + config_.init_width;
  *iter_ring =      *iter_ring + config_.init_width;
  *iter_intensity = *iter_intensity + config_.init_width;
  *iter_time = *iter_time + config_.init_width;
}

void VelodyneContainerOrganized::setup(const VelodyneContainerConfig& config) {
  config_ = config;
  cloud.height = config_.init_height;
  cloud.width = config_.init_width;
}


void VelodyneContainerOrganized::addPoint(float x,
                                          float y,
                                          float z,
                                          uint16_t ring,
                                          uint16_t azimuth,
                                          float distance,
                                          float intensity,
                                          uint64_t time)
{
  /** The laser values are not ordered, the organized structure
     * needs ordered neighbour points. The right order is defined
     * by the laser_ring value.
     * To keep the right ordering, the filtered values are set to
     * NaN.
     */
  if (pointInRange(distance)) {
    std::cerr << "iter_x" << iter_x.get() << std::endl;
    std::cerr << &(*(*iter_x)) << std::endl;
    *(*iter_x + ring) = x;
    *(*iter_y + ring) = y;
    *(*iter_z + ring) = z;
    *(*iter_intensity + ring) = intensity;
    *(*iter_ring + ring) = ring;
    *(*iter_time + ring) = time;
  } else {
    *(*iter_x + ring) = nanf("");
    *(*iter_y + ring) = nanf("");
    *(*iter_z + ring) = nanf("");
    *(*iter_intensity + ring) = nanf("");
    *(*iter_ring + ring) = ring;
    *(*iter_time + ring) = time;
  }
}


