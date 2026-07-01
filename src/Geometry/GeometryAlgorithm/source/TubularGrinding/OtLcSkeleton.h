#pragma once

#include "TubularGrindingCommon.h"
#include <KdTreePointSet.h>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace geoalgo
{
namespace tg
{

struct OtSkeletonState
{
	std::vector<Vec3> samplePositions;
	std::vector<int> sampleToOriginal;
	std::vector<double> sampleMass;
	std::vector<Vec3> originalPositions;
	std::vector<Vec3> anchorPositions;
	std::vector<double> originalMass;
	std::vector<int> originalCluster;
	std::vector<int> sampleParent;
	std::vector<uint8_t> isSampleSkeleton;
	std::vector<uint8_t> isSkeletonPoint;
	std::vector<uint8_t> isFixedPoint;
	std::vector<std::vector<int>> sampleEdges;
	std::unique_ptr<pclalgo::KdTreePointSet> sampleTree;
};

struct OtLcParams
{
	int centerlineIterations = 80;
	double laplacianLambda = 0.1;
	double laplacianAttraction = 0.2;
	double sectionSpacingMm = 2.0;

	double otSampleRate = 0.10;
	double otCostBeta = 3.0;
	int otSinkhornIters = 50;
	double otSinkhornEps = 0.01;
	int otcOuterLoops = 3;
	int otcPreSteps = 3;

	int pointCloudKnnK = 30;
	double pointCloudVoxelSize = 0.0;

	int otLcOuterMaxIters = 40;
	double otLcEnergyEps = 1e-4;

	/// 根点合并下限（0 = 自动：max(40, sampleCount×0.15)）
	int minRootsBySamples = 0;
};

OtLcParams buildOtLcParams(const TubularGrindingParams& params);

bool buildPointCloudKnnDknnAdjacency(
	const std::vector<float>& xyz,
	int k,
	std::vector<std::vector<int>>& outAdjacency,
	std::string* errMsg);

bool extractCenterlineFromOtSkeleton(
	const std::vector<Vec3>& samplePositions,
	const std::vector<std::vector<int>>& sampleEdges,
	std::vector<Vec3>& outPolyline,
	double sectionSpacingMm,
	OtLcGraphDiagnostics* outDiagnostics = nullptr);

} // namespace tg
} // namespace geoalgo
