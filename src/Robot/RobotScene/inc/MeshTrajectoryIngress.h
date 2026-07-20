#ifndef ROBOTSCENE_MESHTRAJECTORYINGRESS_H
#define ROBOTSCENE_MESHTRAJECTORYINGRESS_H

/// @file MeshTrajectoryIngress.h
/// @brief mesh 模型系 RawPath → RawTrajectory（sourceFeatureJson 为 MeshTrajectorySpec JSON）

#include "robot_scene_global.h"

#include "RawTrajectory.h"

#include <string>

#include <FeatureSpec.h>

namespace RobotInstruction
{
struct ROBOT_SCENE_API MeshTrajectoryIngressParams
{
	FrameStrategy frameStrategy = FrameStrategy::SurfaceNormalZ;
};

/// mesh 模型系 RawPath → RawTrajectory（sourceFeatureJson 为 MeshTrajectorySpec JSON）
ROBOT_SCENE_API bool importMeshRawPathToRawTrajectory(const geoalgo::RawPath& path,
													  const std::string& meshTrajectorySpecJson,
													  const MeshTrajectoryIngressParams& params, RawTrajectory& out,
													  std::string* errMsg = nullptr);

} // namespace RobotInstruction

#endif // ROBOTSCENE_MESHTRAJECTORYINGRESS_H
