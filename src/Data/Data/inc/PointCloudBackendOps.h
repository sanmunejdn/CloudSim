#pragma once

#include "data_global.h"

#include <Eigen/Geometry>

#include <cstddef>
#include <string>
#include <vector>

// 前置声明
namespace pclalgo
{
struct ReconstructionConfig;
}

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

DATA_EXPORT bool cropPointCloudByPolyline2D(
	PointCloudBackendData& data,
	const std::vector<float>& polylineScreenXy,
	const double mvpMatrix[16],
	const double modelToWorld[16],
	int viewportWidth,
	int viewportHeight,
	bool keepInside,
	std::string* errMsg = nullptr);

DATA_EXPORT bool collectPointCloudIndicesByPolyline2D(
	const PointCloudBackendData& data,
	const std::vector<float>& polylineScreenXy,
	const double mvpMatrix[16],
	const double modelToWorld[16],
	int viewportWidth,
	int viewportHeight,
	bool keepInside,
	std::vector<std::size_t>& outIndices,
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

// 配置版本API
DATA_EXPORT bool reconstructMeshFromPointCloudPoissonWithConfig(
	const PointCloudBackendData& pointCloud,
	MeshBackendData& meshOut,
	const pclalgo::ReconstructionConfig& config,
	std::string* errMsg = nullptr);

// 兼容旧名
inline bool downsamplePointCloud(
	PointCloudBackendData& data,
	const double voxelSizeMm,
	std::string* errMsg = nullptr)
{
	return downsamplePointCloudVoxel(data, voxelSizeMm, 1U, errMsg);
}

// === vcglib 网格后处理（x64：Data.dll 链接 VcgAlgorithms.dll） ===
// 使用 raw soup 接口，便于宿主异步调用

DATA_EXPORT bool simplifyMesh(
	const std::vector<float>& soupIn,
	std::vector<float>& soupOut,
	int targetFaceCount,
	double qualityThreshold = 0.3,
	std::string* errMsg = nullptr);

DATA_EXPORT bool smoothMesh(
	const std::vector<float>& soupIn,
	std::vector<float>& soupOut,
	int iterations,
	bool useImplicitFairing = false,
	std::string* errMsg = nullptr);

DATA_EXPORT bool repairMesh(
	const std::vector<float>& soupIn,
	std::vector<float>& soupOut,
	bool removeDegenerate = true,
	bool removeDuplicate = true,
	bool removeNonManifold = true,
	std::string* errMsg = nullptr);

DATA_EXPORT bool remeshMeshIsotropic(
	const std::vector<float>& soupIn,
	std::vector<float>& soupOut,
	double targetEdgeLengthMm,
	int iterations = 3,
	std::string* errMsg = nullptr);

DATA_EXPORT bool reconstructMeshFromPointCloudPoissonAndPostProcess(
	const PointCloudBackendData& pointCloud,
	MeshBackendData& meshOut,
	int targetFaceCount = 0,
	bool doRepair = true,
	bool doSmooth = false,
	std::string* errMsg = nullptr);

DATA_EXPORT bool analyzeMeshDefects(
	const std::vector<float>& soupIn,
	std::vector<int>& defectFaceIndices,
	std::vector<float>& defectScores,
	std::vector<int>& defectKinds,
	int& outTotalFaces,
	int& outDefectFaceCount,
	double& outDefectAreaRatio,
	int& outNeedleCount,
	int& outProtrusionCount,
	int& outBoundarySpikeCount,
	double sensitivity,
	int minClusterFaces,
	bool detectNeedle,
	bool detectProtrusion,
	bool detectBoundarySpike,
	std::string* errMsg = nullptr);

} // namespace point_cloud_backend_ops
