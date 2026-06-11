#pragma once

#include "geometry_algorithm_global.h"
#include "ShapeHandle.h"

#include <string>
#include <vector>

namespace geoalgo
{

struct MeshSurfaceReconstructParams
{
	int normalSmoothIterations = 6;
	double featureThresholdC0 = 0.8;
	bool runVcgRepairFirst = true;

	int patchCountHint = 0;
	int samplesPerPatchEdge = 16;

	double blendStripWidth = 0.0;

	double fairingEpsilon = 1e-3;
	int fairingMaxIterations = 50;

	double tessellateLinearDeflectionMm = 0.1;
};

struct MeshSurfaceReconstructReport
{
	int patchCount = 0;
	int junctionCount = 0;
	double maxDeviationMm = 0.0;
	double globalFairingMetric = 0.0;
	double normalSmoothGapVolume = 0.0;
	bool c2BlendSucceeded = false;
};

/**
 * 三角网格 soup → C² 拼接 B 样条 B-rep（单位 mm）
 * 预处理（法矢光顺/修复）由 Data 层在调用前完成
 */
GEOMETRY_ALGORITHM_API bool reconstructBrepFromMeshSoup(
	const std::vector<float>& soup,
	const MeshSurfaceReconstructParams& params,
	ShapeHandle& outShape,
	MeshSurfaceReconstructReport& report,
	std::string* errMsg = nullptr);

} // namespace geoalgo
