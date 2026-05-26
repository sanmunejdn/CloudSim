#pragma once

#include "TrajectoryPipelineTypes.h"
#include "trajectory_algorithm_global.h"

#include <RigidTransform.h>

namespace trajectory_algo
{

TRAJECTORY_ALGORITHM_API engine::RigidTransform rigidDeltaFromTranslate(
	const RobotInstruction::TranslateParams& translate);

TRAJECTORY_ALGORITHM_API engine::RigidTransform rigidDeltaFromRotate(
	const RobotInstruction::RotateParams& rotate);

TRAJECTORY_ALGORITHM_API engine::RigidTransform applyTransformDelta(
	const engine::RigidTransform& target,
	const engine::RigidTransform& delta,
	RobotInstruction::TransformReferenceFrame frame);

} // namespace trajectory_algo
