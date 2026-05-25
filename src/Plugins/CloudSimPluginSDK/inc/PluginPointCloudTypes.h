#pragma once

#include "cloudsim_plugin_sdk_global.h"

#include "PluginPrimitiveTypes.h"

#include <cstddef>
#include <functional>
#include <string>
#include <vector>

class IPluginDocument;
class QString;

struct PluginAxisAlignedBox
{
	PluginVec3 minMm{};
	PluginVec3 maxMm{};
	bool valid = false;
};

struct PluginPointCloudInfo
{
	std::size_t pointCount = 0U;
	PluginAxisAlignedBox bounds{};
	bool hasPerVertexColors = false;
	bool hasPointNormals = false;
};

struct PluginPointCloudMeasure
{
	PluginVec3 centroidMm{};
	PluginAxisAlignedBox bounds{};
	double averageSpacingMm = 0.0;
};

/// 列主序 4×4 刚体变换（mm）
struct PluginMat4
{
	double v[16] = {
		1.0, 0.0, 0.0, 0.0,
		0.0, 1.0, 0.0, 0.0,
		0.0, 0.0, 1.0, 0.0,
		0.0, 0.0, 0.0, 1.0};
};

struct PluginPointCloudJobResult
{
	std::string newBackendId;
	PluginMat4 icpTransform{};
	double rmseMm = 0.0;
	std::size_t pointCountAfter = 0U;
};

struct PluginPointCloudDownsampleVoxelParams
{
	double voxelSizeMm = 2.0;
	unsigned int minPointsPerCell = 1U;
};

struct PluginPointCloudDownsampleRandomParams
{
	double retainedFraction = 0.5;
};

struct PluginPointCloudCropBoxParams
{
	PluginAxisAlignedBox box{};
};

struct PluginPointCloudCropSphereParams
{
	PluginVec3 centerMm{};
	double radiusMm = 10.0;
};

struct PluginPointCloudOutlierParams
{
	double removalPercent = 5.0;
	unsigned int kNeighbors = 24U;
};

struct PluginPointCloudNormalsParams
{
	unsigned int kNeighbors = 12U;
	unsigned int jetDegreeFitting = 2U;
};

struct PluginPointCloudPreprocessParams
{
	double voxelPrefilterMm = 1.0;
	double outlierRemovalPercent = 5.0;
};

struct PluginPointCloudIcpParams
{
	std::string targetBackendIdUtf8;
	bool applyTransformToSource = true;
	int maxIterations = 40;
	double convergenceTransMm = 0.01;
	double maxPairDistanceMm = 0.0;
	std::size_t icpMaxPoints = 4000U;
};

struct PluginPointCloudTpsControlParams
{
	std::vector<std::size_t> controlPointIndices;
	std::vector<float> controlDisplacementXyz;
	double regularizationLambda = 1e-6;
};

struct PluginPointCloudTpsFitParams
{
	std::string targetBackendIdUtf8;
	std::vector<std::size_t> correspondenceIndices;
	double regularizationLambda = 1e-6;
	bool createNewPointCloud = false;
	PluginMeshCreateOptions newObjectOptions{};
};

struct PluginPointCloudReconstructPoissonParams
{
	double spacingMm = 0.0;
	double smAngleDeg = 20.0;
	double smRadiusRel = 30.0;
	double smDistanceRel = 0.375;
	PluginMeshCreateOptions meshOptions{};
};

struct PluginPointCloudReconstructPoissonAutoParams
{
	double voxelPrefilterMm = 1.0;
	double outlierRemovalPercent = 5.0;
	PluginMeshCreateOptions meshOptions{};
};

struct PluginPointCloudReconstructScaleSpaceParams
{
	std::size_t smoothIterations = 4U;
	double meshingRadiusMm = 0.0;
	PluginMeshCreateOptions meshOptions{};
};

struct PluginPointCloudRigidTransformParams
{
	PluginMat4 transform{};
};

using PluginPointCloudFinishedFn = std::function<void(
	bool ok,
	const QString& error,
	const PluginPointCloudJobResult& result)>;
