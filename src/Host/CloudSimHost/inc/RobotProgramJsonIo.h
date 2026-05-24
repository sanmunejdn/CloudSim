#pragma once

#include "cloudsim_host_global.h"

#include <QJsonArray>

class RobotProgramStore;

namespace cloudsim::host {

class IRobotUrdfImportContext;

/// RobotProgramStore ↔ 工程 robotPrograms[]；加载时校验 sceneBackendId 已存在
CLOUDSIM_HOST_EXPORT QJsonArray robotProgramsToJson(const RobotProgramStore& store);
CLOUDSIM_HOST_EXPORT bool robotProgramsFromJson(RobotProgramStore& store, const QJsonArray& programs,
	IRobotUrdfImportContext& ctx, QString* outError = nullptr);

} // namespace cloudsim::host
