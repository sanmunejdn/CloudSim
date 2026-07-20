#ifndef GEOMETRYALGORITHM_MESHFPFHREGIONPARTITION_H
#define GEOMETRYALGORITHM_MESHFPFHREGIONPARTITION_H

/// @file MeshFpfhRegionPartition.h
/// @brief MeshFpfhRegionPartition 接口

#include "TubularGrindingCommon.h"

#include <string>
#include <vector>

namespace geoalgo
{
namespace tg
{
struct MeshFpfhPartitionParams
{
	double featureVoxelMm = 0.0;
	int maxSamplePoints = 0;
	unsigned int fpfhNeighbors = 20U;
	unsigned int saliencyNeighbors = 10U;
	int keypointCount = 0;
	double keypointMinSeparationMm = 0.0;
	double regionGrowDist = 0.0;
	double regionGrowNormalAngleDeg = 45.0;
	int minRegionFaces = 10;
};

bool runMeshFpfhRegionPartition(const IndexedMeshLite& mesh, const MeshFpfhPartitionParams& params,
								std::vector<int>& outFaceRegionId, int& outRegionCount, int& outKeypointCount,
								std::string* errMsg);

} // namespace tg
} // namespace geoalgo

#endif // GEOMETRYALGORITHM_MESHFPFHREGIONPARTITION_H
