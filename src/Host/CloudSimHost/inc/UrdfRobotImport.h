#pragma once

#include "CoreTypes.h"
#include "cloudsim_host_global.h"

namespace cloudsim::host {

class IRobotUrdfImportContext;

/// per-link 网格注册 + FK 绑定位姿；仿真元数据写入 IRobotUrdfImportContext（DocumentPage）
CLOUDSIM_HOST_EXPORT core::RobotRegistrationDto importUrdfRobot(IRobotUrdfImportContext& ctx, const QString& urdfFilePath,
	const core::ImportOptionsDto& options);

} // namespace cloudsim::host
