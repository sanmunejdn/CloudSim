#pragma once

#include "robotwidget_global.h"

#include <QJsonArray>
#include <QJsonObject>
#include <QString>
#include <QVector>
#include <functional>

class IRobotDocumentHost;

namespace RobotProjectIo
{

ROBOTWIDGET_EXPORT void writeRobotKinematics(
	QJsonObject& root,
	IRobotDocumentHost* doc,
	const QVector<double>* aggregatedJointAnglesRad = nullptr);

ROBOTWIDGET_EXPORT void loadRobotPrograms(
	const QJsonObject& root,
	IRobotDocumentHost* doc,
	const std::function<void(const QString&)>& appendWarning);

}
