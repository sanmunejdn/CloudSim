#pragma once

#include "data_global.h"

#include <Eigen/Geometry>
#include <string>
#include <vector>

class PointCloudBackendData;
class MeshBackendData;

namespace point_cloud_backend_ops
{

/// 对后端点云缓冲执行体素下采样并写回。
DATA_EXPORT bool downsamplePointCloud(
	PointCloudBackendData& data,
	double voxelSizeMm,
	std::string* errMsg = nullptr);

/// 将刚性变换应用到点坐标（模型空间，列向量语义）。
DATA_EXPORT bool applyRigidTransformToPointCloud(
	PointCloudBackendData& data,
	const Eigen::Isometry3d& transform,
	std::string* errMsg = nullptr);

/// 从点云重建三角网格 soup，写入 MeshBackendData（每三角 9 float）。
DATA_EXPORT bool reconstructMeshFromPointCloudPoisson(
	const PointCloudBackendData& pointCloud,
	MeshBackendData& meshOut,
	double voxelPrefilterMm,
	std::string* errMsg = nullptr);

} // namespace point_cloud_backend_ops
