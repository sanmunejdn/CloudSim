#ifndef TRAJECTORYALGORITHM_TRAJECTORYOPEXECUTIONCONTEXT_H
#define TRAJECTORYALGORITHM_TRAJECTORYOPEXECUTIONCONTEXT_H

/// @file TrajectoryOpExecutionContext.h
/// @brief 几何投影/非刚性/工件参考位姿：由 RobotScene 注入，Builtins 不依赖具体解析实现

#include "trajectory_algorithm_global.h"

#include "INonRigidTrajectoryWarp.h"
#include "TrajectoryPipelineTypes.h"

#include <RigidTransform.h>

#include <cstddef>
#include <string>

namespace RobotInstruction
{
struct UnifiedTrajectory;
class RobotProgram;
} // namespace RobotInstruction

namespace trajectory_algo
{
/// 几何投影服务：由 RobotScene 注入，Builtins 不依赖具体解析实现
class TRAJECTORY_ALGORITHM_API IGeometryProjection
{
public:
	virtual ~IGeometryProjection() = default;

	virtual bool project(RobotInstruction::UnifiedTrajectory& traj,
						 const RobotInstruction::ProjectToGeometryParams& params,
						 const RobotInstruction::OpScope& scope, const RobotInstruction::RobotProgram* program,
						 std::size_t* missCount, std::string* errMsg) const = 0;
};

struct TRAJECTORY_ALGORITHM_API TrajectoryOpExecutionContext
{
	const RobotInstruction::RobotProgram* program = nullptr;
	const IGeometryProjection* geometryProjection = nullptr;
	const INonRigidTrajectoryWarp* nonRigidTrajectoryWarp = nullptr;
	/// 当前工具 TCP 在机器人基座系（工件型转换用）
	bool hasWorkpieceReferenceInBase = false;
	engine::RigidTransform workpieceReferenceInBase{};
	/// 外部 TCP 来自 Frame 后端世界位姿（优先于手动六参数）
	bool hasExternalTcpFromBackend = false;
	engine::RigidTransform externalTcpInBase{};
};

} // namespace trajectory_algo

#endif // TRAJECTORYALGORITHM_TRAJECTORYOPEXECUTIONCONTEXT_H
