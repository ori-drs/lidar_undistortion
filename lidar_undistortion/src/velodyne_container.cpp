#include "lidar_undistortion/velodyne_container.hpp"
#include "lidar_undistortion/velodyne_point.hpp"

using namespace velodyne;

void VelodyneContainer::addPoint(float x,
                                 float y,
                                 float z,
                                 uint16_t ring,
                                 uint16_t /*azimuth*/,
                                 float /*distance*/,
                                 float intensity,
                                 uint64_t time)
{
  // convert polar coordinates to Euclidean XYZ
  velodyne::PointXYZIRT point;
  point.ring = ring;
  point.x = x;
  point.y = y;
  point.z = z;
  point.intensity = intensity;
  point.time = time;

  // append this point to the cloud
  pc->points.push_back(point);
  ++pc->width;
}



