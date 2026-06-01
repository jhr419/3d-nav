#ifndef DDDMR_FASTLIO_ADAPTER_POINT_TYPES_HPP_
#define DDDMR_FASTLIO_ADAPTER_POINT_TYPES_HPP_

#include <pcl/point_types.h>

struct PointXYZIRPYT
{
  PCL_ADD_POINT4D
  PCL_ADD_INTENSITY;
  float roll;
  float pitch;
  float yaw;
  double time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(
  PointXYZIRPYT,
  (float, x, x)
  (float, y, y)
  (float, z, z)
  (float, intensity, intensity)
  (float, roll, roll)
  (float, pitch, pitch)
  (float, yaw, yaw)
  (double, time, time)
)

using PointTypePose = PointXYZIRPYT;
using PointType = pcl::PointXYZI;

#endif
