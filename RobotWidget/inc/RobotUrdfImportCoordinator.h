#pragma once

#include "robotwidget_global.h"

#include <QString>

class IRobotMainWindowHost;

namespace RobotUrdfImport
{

ROBOTWIDGET_EXPORT bool registerUrdfRobot(IRobotMainWindowHost* host, const QString& urdfPath, bool quietUi);

}
