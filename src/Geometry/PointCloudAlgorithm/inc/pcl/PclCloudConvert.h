#ifndef POINTCLOUDALGORITHM_PCL_CLOUDCONVERT_H
#define POINTCLOUDALGORITHM_PCL_CLOUDCONVERT_H

/// @file PclCloudConvert.h
/// @brief xyz/法线缓冲 ↔ PCL PointNormal（仅 CLOUDSIM_HAS_PCL）

#ifdef CLOUDSIM_HAS_PCL

#include <memory>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace pclalgo
{
namespace pcl_bridge
{
using PointNT = pcl::PointNormal;
using CloudNT = pcl::PointCloud<PointNT>;
using CloudNTPtr = CloudNT::Ptr;

CloudNTPtr makeCloudFromXyzNormals(const std::vector<float>& xyz, const std::vector<float>& normalsNxNyNz);

bool cloudHasUsableNormals(const CloudNT& cloud);

} // namespace pcl_bridge
} // namespace pclalgo

#endif // CLOUDSIM_HAS_PCL

#endif // POINTCLOUDALGORITHM_PCL_CLOUDCONVERT_H
