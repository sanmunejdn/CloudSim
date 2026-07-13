#pragma once

#include "point_cloud_algorithm_global.h"

#include <cstddef>
#include <string>
#include <vector>

namespace pclalgo
{

struct SpareRegisterParams
{
	double sampleRadiusRatio = 0.0;
	double wSmo = 0.01;
	double wRot = 1e-4;
	double wArapCoarse = 500.0;
	double wArapFine = 200.0;
	bool useSymmetricPointToPlane = true;
	bool useCoarseReg = true;
	bool useFineReg = true;
	bool normalizeScale = true;
	bool rigidPreAlign = false;
	bool coarseGlobalAlign = false;
	double voxelPrefilterMm = 0.0;
	int maxOuterIters = 30;
	double stopCoarse = 1e-3;
	double stopFine = 1e-4;
	std::size_t alignSampleCount = 3000U;
	int rigidPreAlignMaxIterations = 40;
	double rigidPreAlignMaxPairDistanceMm = 0.0;
	std::size_t rigidPreAlignMaxPoints = 4000U;
};

struct SpareRegisterResult
{
	double meanErrorMm = 0.0;
	double meshScale = 1.0;
	int deformationNodeCount = 0;
};

POINT_CLOUD_ALGORITHM_API bool spareRegisterPointClouds(
	const std::vector<float>& sourceXyz,
	const std::vector<float>& sourceNormals,
	const std::vector<float>& targetXyz,
	const std::vector<float>& targetNormals,
	std::vector<float>& sourceXyzDeformedOut,
	std::vector<float>& sourceNormalsDeformedOut,
	const SpareRegisterParams& params,
	SpareRegisterResult* stats,
	std::string* errMsg);

POINT_CLOUD_ALGORITHM_API bool spareRegisterMeshSoupToTarget(
	const std::vector<float>& sourceSoup,
	const std::vector<float>& targetXyz,
	const std::vector<float>& targetNormals,
	std::vector<float>& sourceSoupDeformedOut,
	const SpareRegisterParams& params,
	SpareRegisterResult* stats,
	std::string* errMsg);

POINT_CLOUD_ALGORITHM_API bool spareRegisterMeshSoupToMeshSoup(
	const std::vector<float>& sourceSoup,
	const std::vector<float>& targetSoup,
	std::vector<float>& sourceSoupDeformedOut,
	const SpareRegisterParams& params,
	SpareRegisterResult* stats,
	std::string* errMsg);

} // namespace pclalgo
