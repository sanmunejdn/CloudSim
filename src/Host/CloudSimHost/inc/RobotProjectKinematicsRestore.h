#pragma once

#include "cloudsim_host_global.h"

#include <QJsonObject>
#include <QSet>
#include <QVector>

namespace cloudsim::host {

class IRobotUrdfImportContext;

/// 收集 perLink 连杆网格 id，工程加载时跳过 robotLink 的重复文件导入
CLOUDSIM_HOST_EXPORT QSet<QString> collectRobotLinkMeshBackendIds(const QJsonObject& projectRoot);

/// 恢复单条 robotKinematics（mode=perLink）；成功时追加 jointAnglesRad 到 outAllJointAnglesRad
CLOUDSIM_HOST_EXPORT bool restorePerLinkRobotKinematicsFromProjectJson(IRobotUrdfImportContext& ctx,
	const QJsonObject& robotKinematicsJson, QVector<double>& outAllJointAnglesRad, QString* outWarning = nullptr);

} // namespace cloudsim::host
