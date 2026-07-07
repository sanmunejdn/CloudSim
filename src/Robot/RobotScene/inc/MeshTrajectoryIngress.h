#pragma once

#include "robot_scene_global.h"
#include "RawTrajectory.h"

#include <FeatureSpec.h>

#include <string>

namespace RobotInstruction
{

struct ROBOT_SCENE_API MeshTrajectoryIngressParams
{
	FrameStrategy frameStrategy = FrameStrategy::SurfaceNormalZ;
};

/// mesh 模型系 RawPath → RawTrajectory（sourceFeatureJson 为 MeshTrajectorySpec JSON）
ROBOT_SCENE_API bool importMeshRawPathToRawTrajectory(
	const geoalgo::RawPath& path,
	const std::string& meshTrajectorySpecJson,
	const MeshTrajectoryIngressParams& params,
	RawTrajectory& out,
	std::string* errMsg = nullptr);

} // namespace RobotInstruction
