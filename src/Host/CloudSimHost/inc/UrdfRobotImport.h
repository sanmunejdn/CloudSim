#pragma once

#include "CoreTypes.h"
#include "cloudsim_host_global.h"

namespace cloudsim::host {

class IRobotUrdfImportContext;

/// URDF 导入注册
CLOUDSIM_HOST_EXPORT core::RobotRegistrationDto importUrdfRobot(IRobotUrdfImportContext& ctx, const QString& urdfFilePath,
	const core::ImportOptionsDto& options);

} // namespace cloudsim::host
