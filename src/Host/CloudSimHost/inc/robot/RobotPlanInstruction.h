#ifndef CLOUDSIMHOST_ROBOTPLANINSTRUCTION_H
#define CLOUDSIMHOST_ROBOTPLANINSTRUCTION_H

/// @file RobotPlanInstruction.h
/// @brief DTO 转规划指令

#include "cloudsim_host_global.h"

#include "CoreTypes.h"

#include <QVector>
#include <string>

namespace RobotInstruction
{
class Base;
struct PlanResult;
} // namespace RobotInstruction

namespace cloudsim::host
{
class IRobotUrdfImportContext;

/// DTO 转规划指令
CLOUDSIM_HOST_EXPORT bool planMotionInstruction(IRobotUrdfImportContext& ctx,
												const core::MotionInstructionDto& instruction,
												const core::PlanContextDto& context, core::PlanResultDto& out,
												QString* outError = nullptr);

/// 已 prepare 指令规划
CLOUDSIM_HOST_EXPORT bool planRobotInstruction(IRobotUrdfImportContext& ctx, RobotInstruction::Base& instruction,
											   const QVector<double>& seedJointRad, int instanceIndex,
											   const QString& urdfPath, const std::string& defaultTcpLinkName,
											   const QString& sceneRootBackendId, RobotInstruction::PlanResult& out,
											   QString* outError = nullptr);

} // namespace cloudsim::host

#endif // CLOUDSIMHOST_ROBOTPLANINSTRUCTION_H
