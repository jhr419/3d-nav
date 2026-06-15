#pragma once

#include <builtin_interfaces/msg/time.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <memory>
#include <cmath>
#include <vector>

namespace sc_lio_sam_backend
{

using PointT = pcl::PointXYZI;
using PointCloud = pcl::PointCloud<PointT>;
using PointCloudPtr = PointCloud::Ptr;
using PointCloudConstPtr = PointCloud::ConstPtr;
using ScanContextDescriptor = Eigen::MatrixXd;

struct Keyframe
{
  int id = -1;
  builtin_interfaces::msg::Time stamp;
  double stamp_sec = 0.0;
  Eigen::Isometry3d raw_pose = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d optimized_pose = Eigen::Isometry3d::Identity();
  PointCloudPtr cloud;
  ScanContextDescriptor scan_context;
};

struct LoopEdge
{
  int from = -1;
  int to = -1;
  double fitness = 0.0;
};

inline double translationDistance(
  const Eigen::Isometry3d & a,
  const Eigen::Isometry3d & b)
{
  return (a.translation() - b.translation()).norm();
}

inline double rotationAngleDeg(
  const Eigen::Isometry3d & a,
  const Eigen::Isometry3d & b)
{
  const Eigen::Matrix3d rel = a.linear().transpose() * b.linear();
  const Eigen::AngleAxisd aa(rel);
  return std::abs(aa.angle()) * 180.0 / M_PI;
}

}  // namespace sc_lio_sam_backend
