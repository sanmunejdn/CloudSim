#pragma once

#include "cloudsim_host_global.h"

#include <QJsonObject>
#include <QSet>
#include <QVector>

namespace cloudsim::host {

class IRobotUrdfImportContext;

/// 收集连杆网格 id
CLOUDSIM_HOST_EXPORT QSet<QString> collectRobotLinkMeshBackendIds(const QJsonObject& projectRoot);

/// 恢复 perLink 运动学
CLOUDSIM_HOST_EXPORT bool restorePerLinkRobotKinematicsFromProjectJson(IRobotUrdfImportContext& ctx,
	const QJsonObject& robotKinematicsJson, QVector<double>& outAllJointAnglesRad, QString* outWarning = nullptr);

} // namespace cloudsim::host
