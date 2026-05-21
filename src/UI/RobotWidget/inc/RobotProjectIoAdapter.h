#pragma once

#include "robotwidget_global.h"

#include <QJsonArray>
#include <QJsonObject>
#include <QString>
#include <functional>

class IRobotDocumentHost;

namespace RobotProjectIo
{

ROBOTWIDGET_EXPORT void writeRobotKinematicsAndPrograms(QJsonObject& root, IRobotDocumentHost* doc);
ROBOTWIDGET_EXPORT void loadRobotPrograms(
	const QJsonObject& root,
	IRobotDocumentHost* doc,
	const std::function<void(const QString&)>& appendWarning);

}
