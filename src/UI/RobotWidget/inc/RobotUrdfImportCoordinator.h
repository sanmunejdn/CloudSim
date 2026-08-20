#ifndef ROBOTWIDGET_ROBOTURDFIMPORTCOORDINATOR_H
#define ROBOTWIDGET_ROBOTURDFIMPORTCOORDINATOR_H

/// @file RobotUrdfImportCoordinator.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief RobotUrdfImportCoordinator 接口

#include "robotwidget_global.h"

#include <QString>

class IRobotMainWindowHost;

namespace RobotUrdfImport
{
ROBOTWIDGET_EXPORT bool registerUrdfRobot(IRobotMainWindowHost* host, const QString& urdfPath, bool quietUi);

}

#endif // ROBOTWIDGET_ROBOTURDFIMPORTCOORDINATOR_H
