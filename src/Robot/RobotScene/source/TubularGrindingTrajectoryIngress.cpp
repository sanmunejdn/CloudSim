/// @file TubularGrindingTrajectoryIngress.cpp
/// @brief 管状打磨轨迹入口

#include "TubularGrindingTrajectoryIngress.h"

namespace RobotInstruction
{
bool importTubularGrindingPointsToRawTrajectory(const geoalgo::TubularGrindingProjectedPoints& points,
												const TubularGrindingTrajectoryIngressParams& params,
												RawTrajectory& out, std::string* errMsg)
{
	(void)points;
	(void)params;
	out = RawTrajectory{};
	if (errMsg)
	{
		*errMsg = "not implemented";
	}
	return false;
}

} // namespace RobotInstruction
