#pragma once

#include "INonRigidTrajectoryWarp.h"
#include "TrajectoryPipelineTypes.h"
#include "trajectory_algorithm_global.h"

#include <cstddef>
#include <string>

namespace RobotInstruction
{
struct UnifiedTrajectory;
class RobotProgram;
}

namespace trajectory_algo
{

/// 几何投影服务：由 RobotScene 注入，Builtins 不依赖具体解析实现
class TRAJECTORY_ALGORITHM_API IGeometryProjection
{
public:
	virtual ~IGeometryProjection() = default;

	virtual bool project(
		RobotInstruction::UnifiedTrajectory& traj,
		const RobotInstruction::ProjectToGeometryParams& params,
		const RobotInstruction::OpScope& scope,
		const RobotInstruction::RobotProgram* program,
		std::size_t* missCount,
		std::string* errMsg) const = 0;
};

struct TRAJECTORY_ALGORITHM_API TrajectoryOpExecutionContext
{
	const RobotInstruction::RobotProgram* program = nullptr;
	const IGeometryProjection* geometryProjection = nullptr;
	const INonRigidTrajectoryWarp* nonRigidTrajectoryWarp = nullptr;
};

} // namespace trajectory_algo
