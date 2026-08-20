#ifndef ROBOTSCENE_MESHTRAJECTORYINGRESS_H
#define ROBOTSCENE_MESHTRAJECTORYINGRESS_H

/// @file MeshTrajectoryIngress.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
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
