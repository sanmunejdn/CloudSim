#ifndef CLOUDSIMHOST_URDFROBOTIMPORT_H
#define CLOUDSIMHOST_URDFROBOTIMPORT_H

/// @file UrdfRobotImport.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief URDF 导入注册

#include "cloudsim_host_global.h"

#include "CoreTypes.h"

namespace cloudsim::host
{
class IRobotUrdfImportContext;

/// URDF 导入注册
CLOUDSIM_HOST_EXPORT core::RobotRegistrationDto
importUrdfRobot(IRobotUrdfImportContext& ctx, const QString& urdfFilePath, const core::ImportOptionsDto& options);

} // namespace cloudsim::host

#endif // CLOUDSIMHOST_URDFROBOTIMPORT_H
