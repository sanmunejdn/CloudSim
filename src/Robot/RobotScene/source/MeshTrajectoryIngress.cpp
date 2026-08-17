/// @file MeshTrajectoryIngress.cpp
/// @brief 网格轨迹入口

#include "MeshTrajectoryIngress.h"

#include "RawTrajectory.h"

namespace RobotInstruction
{
bool importMeshRawPathToRawTrajectory(const geoalgo::RawPath& path, const std::string& meshTrajectorySpecJson,
									  const MeshTrajectoryIngressParams& params, RawTrajectory& out,
									  std::string* errMsg)
{
	out = RawTrajectory{};
	if (path.points.size() < 2U)
	{
		if (errMsg)
		{
			*errMsg = "轨迹点数不足";
		}
		return false;
	}
	if (!importRawPathToTrajectory(path, params.frameStrategy, out, errMsg))
	{
		return false;
	}
	out.sourceFeatureJson = meshTrajectorySpecJson;
	return true;
}

} // namespace RobotInstruction
