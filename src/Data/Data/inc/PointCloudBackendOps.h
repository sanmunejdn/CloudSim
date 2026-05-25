#pragma once

#include "data_global.h"

#include <Eigen/Geometry>

#include <cstddef>
#include <string>
#include <vector>

class PointCloudBackendData;
class MeshBackendData;

namespace point_cloud_backend_ops
{

struct PointCloudMeasureResult
{
	Eigen::Vector3d centroidMm{ Eigen::Vector3d::Zero() };
	Eigen::AlignedBox3d boundingBoxMm;
	double averageSpacingMm = 0.0;
};

struct PointCloudIcpResult
{
	Eigen::Isometry3d sourceToTarget{ Eigen::Isometry3d::Identity() };
	double rmseMm = 0.0;
};

DATA_EXPORT bool downsamplePointCloudVoxel(
	PointCloudBackendData& data,
	double voxelSizeMm,
	unsigned int minPointsPerCell,
	std::string* errMsg = nullptr);

DATA_EXPORT bool downsamplePointCloudRandom(
	PointCloudBackendData& data,
	double retainedFraction,
	std::string* errMsg = nullptr);

DATA_EXPORT bool applyRigidTransformToPointCloud(
	PointCloudBackendData& data,
	const Eigen::Isometry3d& transform,
	std::string* errMsg = nullptr);

DATA_EXPORT bool cropPointCloudByBox(
	PointCloudBackendData& data,
	const Eigen::AlignedBox3d& box,
	std::string* errMsg = nullptr);

DATA_EXPORT bool cropPointCloudBySphere(
	PointCloudBackendData& data,
	const Eigen::Vector3d& centerMm,
	double radiusMm,
	std::string* errMsg = nullptr);

DATA_EXPORT bool measurePointCloud(
	const PointCloudBackendData& data,
	PointCloudMeasureResult& out,
	std::string* errMsg = nullptr);

DATA_EXPORT bool removePointCloudOutliers(
	PointCloudBackendData& data,
	double removalPercent,
	unsigned int kNeighbors,
	std::string* errMsg = nullptr);

DATA_EXPORT bool smoothPointCloudBilateral(
	PointCloudBackendData& data,
	std::string* errMsg = nullptr);

DATA_EXPORT bool estimatePointCloudNormalsPca(
	PointCloudBackendData& data,
	unsigned int kNeighbors,
	std::string* errMsg = nullptr);

DATA_EXPORT bool estimatePointCloudNormalsJet(
	PointCloudBackendData& data,
	unsigned int kNeighbors,
	unsigned int degreeFitting,
	std::string* errMsg = nullptr);

DATA_EXPORT bool orientPointCloudNormalsMst(
	PointCloudBackendData& data,
	unsigned int kNeighbors,
	std::string* errMsg = nullptr);

DATA_EXPORT bool preprocessPointCloudForReconstruction(
	PointCloudBackendData& data,
	double voxelPrefilterMm,
	double outlierRemovalPercent,
	std::string* errMsg = nullptr);

DATA_EXPORT bool rigidRegisterPointCloudsIcp(
	const PointCloudBackendData& source,
	const PointCloudBackendData& target,
	PointCloudIcpResult& out,
	int maxIterations,
	double convergenceTransMm,
	double maxPairDistanceMm,
	std::size_t icpMaxPoints,
	std::string* errMsg = nullptr);

DATA_EXPORT bool deformPointCloudTpsFromControls(
	PointCloudBackendData& data,
	const std::vector<std::size_t>& controlPointIndices,
	const std::vector<float>& controlDisplacementXyz,
	double regularizationLambda,
	std::string* errMsg = nullptr);

DATA_EXPORT bool deformPointCloudTpsFitAndDeform(
	const PointCloudBackendData& source,
	const PointCloudBackendData& target,
	const std::vector<std::size_t>& correspondenceIndices,
	std::vector<float>& sourceXyzDeformedOut,
	double regularizationLambda,
	std::string* errMsg = nullptr);

DATA_EXPORT bool reconstructMeshPoisson(
	const PointCloudBackendData& pointCloud,
	MeshBackendData& meshOut,
	double spacingMm,
	double smAngleDeg,
	double smRadiusRel,
	double smDistanceRel,
	std::string* errMsg = nullptr);

DATA_EXPORT bool reconstructMeshFromPointCloudPoisson(
	const PointCloudBackendData& pointCloud,
	MeshBackendData& meshOut,
	double voxelPrefilterMm,
	std::string* errMsg = nullptr);

DATA_EXPORT bool reconstructMeshScaleSpace(
	const PointCloudBackendData& pointCloud,
	MeshBackendData& meshOut,
	std::size_t smoothIterations,
	double meshingRadiusMm,
	std::string* errMsg = nullptr);

// 兼容旧名
inline bool downsamplePointCloud(
	PointCloudBackendData& data,
	const double voxelSizeMm,
	std::string* errMsg = nullptr)
{
	return downsamplePointCloudVoxel(data, voxelSizeMm, 1U, errMsg);
}

} // namespace point_cloud_backend_ops
