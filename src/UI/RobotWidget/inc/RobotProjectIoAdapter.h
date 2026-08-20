#ifndef ROBOTWIDGET_ROBOTPROJECTIOADAPTER_H
#define ROBOTWIDGET_ROBOTPROJECTIOADAPTER_H

/// @file RobotProjectIoAdapter.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief RobotProjectIoAdapter 接口

#include "robotwidget_global.h"

#include <QJsonArray>
#include <QJsonObject>
#include <QString>
#include <QVector>
#include <functional>

class IRobotDocumentHost;

namespace RobotProjectIo
{
ROBOTWIDGET_EXPORT void writeRobotKinematics(QJsonObject& root, IRobotDocumentHost* doc,
											 const QVector<double>* aggregatedJointAnglesRad = nullptr);

ROBOTWIDGET_EXPORT void loadRobotPrograms(const QJsonObject& root, IRobotDocumentHost* doc,
										  const std::function<void(const QString&)>& appendWarning);

} // namespace RobotProjectIo

#endif // ROBOTWIDGET_ROBOTPROJECTIOADAPTER_H
