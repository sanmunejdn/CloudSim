#ifndef POINTCLOUDALGORITHM_REGISTRATIONSDF_H
#define POINTCLOUDALGORITHM_REGISTRATIONSDF_H

/// @file RegistrationSdf.h
/// @brief SDF/DDF 混合非刚性配准：粗阶段场残差 + 细阶段默认点-面（独立于 SPARE）

#include "point_cloud_algorithm_global.h"

#include <cstddef>
#include <string>
#include <vector>

namespace pclalgo
{

enum class SdfFieldMode : int
{
	DdfVector = 0,
	SignedDistance = 1,
};

enum class SdfFineDataTerm : int
{
	PointToPlane = 0,
	DdfVector = 1,
	SignedDistance = 2,
};

struct SdfRegisterParams
{
	SdfFieldMode fieldMode = SdfFieldMode::DdfVector;
	double fieldVoxelMm = 0.0; ///< 0=自动（约平均点距）
	SdfFineDataTerm fineDataTerm = SdfFineDataTerm::PointToPlane;
	bool useCoarseReg = true;
	bool useFineReg = true;
	double sampleRadiusRatio = 0.0; ///< 0=自动
	double wSmo = 0.01;
	double wRot = 1e-4;
	double wArapCoarse = 500.0;
	double wArapFine = 200.0;
	bool normalizeScale = true;
	bool rigidPreAlign = false;
	double voxelPrefilterMm = 0.0;
	int maxOuterIters = 30;
	double stopCoarse = 1e-3;
	double stopFine = 1e-4;
	std::size_t alignSampleCount = 3000U;
	int rigidPreAlignMaxIterations = 40;
	double rigidPreAlignMaxPairDistanceMm = 0.0;
	std::size_t rigidPreAlignMaxPoints = 4000U;
	int fieldRebuildEvery = 0; ///< 0=不刷新体素场
};

struct SdfRegisterResult
{
	double meanErrorMm = 0.0;
	double meshScale = 1.0;
	int deformationNodeCount = 0;
	double fieldVoxelMmUsed = 0.0;
};

POINT_CLOUD_ALGORITHM_API bool
sdfRegisterPointClouds(const std::vector<float>& sourceXyz, const std::vector<float>& sourceNormals,
					   const std::vector<float>& targetXyz, const std::vector<float>& targetNormals,
					   std::vector<float>& sourceXyzDeformedOut, std::vector<float>& sourceNormalsDeformedOut,
					   const SdfRegisterParams& params, SdfRegisterResult* stats, std::string* errMsg);

POINT_CLOUD_ALGORITHM_API bool
sdfRegisterMeshSoupToTarget(const std::vector<float>& sourceSoup, const std::vector<float>& targetXyz,
							const std::vector<float>& targetNormals, std::vector<float>& sourceSoupDeformedOut,
							const SdfRegisterParams& params, SdfRegisterResult* stats, std::string* errMsg);

POINT_CLOUD_ALGORITHM_API bool sdfRegisterMeshSoupToMeshSoup(const std::vector<float>& sourceSoup,
															 const std::vector<float>& targetSoup,
															 std::vector<float>& sourceSoupDeformedOut,
															 const SdfRegisterParams& params, SdfRegisterResult* stats,
															 std::string* errMsg);

} // namespace pclalgo

#endif // POINTCLOUDALGORITHM_REGISTRATIONSDF_H
