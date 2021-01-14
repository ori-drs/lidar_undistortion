#pragma once
#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>


namespace lidar_undistortion {

struct EIGEN_ALIGN16 PointOuster {
    PCL_ADD_POINT4D;
    PCL_ADD_INTENSITY;
    uint32_t t;
    uint16_t reflectivity;
    uint8_t ring;
    uint16_t ambient;
    uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};
}  // namespace ouster_ros

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(lidar_undistortion::PointOuster,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (uint32_t, t, t)
    (uint16_t, reflectivity, reflectivity)
    (uint8_t, ring, ring)
    (uint16_t, ambient, ambient)
    (uint32_t, range, range)
)
// clang-format on
