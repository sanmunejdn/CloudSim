#ifndef ROBOTSCENE_TUBULARGRINDINGTRAJECTORYINGRESS_H
#define ROBOTSCENE_TUBULARGRINDINGTRAJECTORYINGRESS_H

/// @file TubularGrindingTrajectoryIngress.h
/// @brief 预留：管状打磨投影点位 → RawTrajectory（Phase 5 进给/工具轴时再补全）

#include "robot_scene_global.h"

#include "RawTrajectory.h"

#include <string>

#include <TubularGrinding.h>

namespace RobotInstruction
{
/// 预留：管状打磨投影点位 → RawTrajectory（Phase 5 进给/工具轴时再补全）
struct ROBOT_SCENE_API TubularGrindingTrajectoryIngressParams
{
	double defaultFeedMmMin = 100.0;
	FrameStrategy frameStrategy = FrameStrategy::SurfaceNormalZ;
};

ROBOT_SCENE_API bool importTubularGrindingPointsToRawTrajectory(const geoalgo::TubularGrindingProjectedPoints& points,
																const TubularGrindingTrajectoryIngressParams& params,
																RawTrajectory& out, std::string* errMsg = nullptr);

} // namespace RobotInstruction

#endif // ROBOTSCENE_TUBULARGRINDINGTRAJECTORYINGRESS_H
