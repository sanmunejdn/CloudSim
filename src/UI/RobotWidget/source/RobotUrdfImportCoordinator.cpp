/// @file RobotUrdfImportCoordinator.cpp
/// @brief RobotUrdfImportCoordinator 实现

#include "RobotUrdfImportCoordinator.h"

#include "IRobotMainWindowHost.h"

namespace RobotUrdfImport
{
bool registerUrdfRobot(IRobotMainWindowHost* host, const QString& urdfPath, const bool quietUi)
{
	return host && host->registerUrdfRobot(urdfPath, quietUi);
}

} // namespace RobotUrdfImport
