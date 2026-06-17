#pragma once

#include "MeshSurfaceReconstructionPartitionCommon.h"

#include <vector>

namespace geoalgo
{
namespace meshrecon
{

struct HybridAdjustStats
{
	int triPatchCount = 0;
	int quadPatchCount = 0;
	int pentPatchCount = 0;
	int hexPatchCount = 0;
};

/// 论文 §3.3：广义边 dual-graph 四边区域调整
bool hybridApplyRegionAdjust(
	const IndexedMeshLite& mesh,
	const MeshSurfaceReconstructParams& params,
	const std::vector<std::vector<int>>& fullAdj,
	const std::vector<PartitionVec3d>& faceNormals,
	std::vector<int>& faceToPatch,
	HybridAdjustStats& stats,
	std::string* errMsg);

} // namespace meshrecon
} // namespace geoalgo
