#ifndef TRAJECTORYALGORITHM_TRAJECTORYOPEXECUTIONCONTEXT_H
#define TRAJECTORYALGORITHM_TRAJECTORYOPEXECUTIONCONTEXT_H

/// @file TrajectoryOpExecutionContext.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 几何投影/非刚性/工件参考位姿：由 RobotScene 注入，Builtins 不依赖具体解析实现

#include "trajectory_algorithm_global.h"

#include "INonRigidTrajectoryWarp.h"
#include "IExternalAxisSearchService.h"
#include "ITrajectoryReachabilityProbe.h"
#include "TrajectoryPipelineTypes.h"

#include <RigidTransform.h>

#include <cstddef>
#include <string>
#include <vector>

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
	/// 机器人对象已启用的外轴；空则 ExternalAxisSearch 跳过
	std::vector<ExternalAxisSearchConfigDto> externalAxisConfigs;
	const IExternalAxisSearchService* externalAxisSearch = nullptr;
	bool externalAxisAllowCoupledRefine = true;
	/// 未注入时 ReachabilityFilter 失败（不再用 z 启发式）
	const ITrajectoryReachabilityProbe* reachabilityProbe = nullptr;
};

} // namespace trajectory_algo

#endif // TRAJECTORYALGORITHM_TRAJECTORYOPEXECUTIONCONTEXT_H
