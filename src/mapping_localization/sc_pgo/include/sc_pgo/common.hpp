#pragma once

#include <cmath>

#include <pcl/point_types.h>

namespace sc_pgo
{

using PointType = pcl::PointXYZI;

struct Pose6D
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
};

inline double deg2rad(const double degrees)
{
  return degrees * M_PI / 180.0;
}

inline double rad2deg(const double radians)
{
  return radians * 180.0 / M_PI;
}

}  // namespace sc_pgo
