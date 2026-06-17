#pragma once

#include "robot_scene_global.h"
#include "RawTrajectory.h"

#include <TubularGrinding.h>

#include <string>

namespace RobotInstruction
{

/// 预留：管状打磨投影点位 → RawTrajectory（Phase 5 进给/工具轴时再补全）
struct ROBOT_SCENE_API TubularGrindingTrajectoryIngressParams
{
	double defaultFeedMmMin = 100.0;
	FrameStrategy frameStrategy = FrameStrategy::SurfaceNormalZ;
};

ROBOT_SCENE_API bool importTubularGrindingPointsToRawTrajectory(
	const geoalgo::TubularGrindingProjectedPoints& points,
	const TubularGrindingTrajectoryIngressParams& params,
	RawTrajectory& out,
	std::string* errMsg = nullptr);

} // namespace RobotInstruction
