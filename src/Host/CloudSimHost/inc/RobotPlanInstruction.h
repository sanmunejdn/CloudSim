#pragma once

#include "CoreTypes.h"
#include "cloudsim_host_global.h"

#include <QVector>
#include <string>

namespace RobotInstruction {
class Base;
struct PlanResult;
}

namespace cloudsim::host {

class IRobotUrdfImportContext;

/// MotionInstructionDto → RobotInstruction 规划；完整示教上下文仍依赖 extensions
CLOUDSIM_HOST_EXPORT bool planMotionInstruction(IRobotUrdfImportContext& ctx, const core::MotionInstructionDto& instruction,
	const core::PlanContextDto& context, core::PlanResultDto& out, QString* outError = nullptr);

/// 已 prepare 的指令 → Host 规划（供 RobotSimulationController 经 MainWindow 调用）
CLOUDSIM_HOST_EXPORT bool planRobotInstruction(IRobotUrdfImportContext& ctx, RobotInstruction::Base& instruction,
	const QVector<double>& seedJointRad, int instanceIndex, const QString& urdfPath, const std::string& defaultTcpLinkName,
	const QString& sceneRootBackendId, RobotInstruction::PlanResult& out, QString* outError = nullptr);

} // namespace cloudsim::host
