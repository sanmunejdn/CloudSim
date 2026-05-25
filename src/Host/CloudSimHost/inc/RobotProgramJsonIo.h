#pragma once

#include "cloudsim_host_global.h"

#include <QJsonArray>

class RobotProgramStore;

namespace cloudsim::host {

class IRobotUrdfImportContext;

/// 机器人程序 JSON
CLOUDSIM_HOST_EXPORT QJsonArray robotProgramsToJson(const RobotProgramStore& store);
CLOUDSIM_HOST_EXPORT bool robotProgramsFromJson(RobotProgramStore& store, const QJsonArray& programs,
	IRobotUrdfImportContext& ctx, QString* outError = nullptr);

} // namespace cloudsim::host
