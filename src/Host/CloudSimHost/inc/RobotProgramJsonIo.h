#ifndef CLOUDSIMHOST_ROBOTPROGRAMJSONIO_H
#define CLOUDSIMHOST_ROBOTPROGRAMJSONIO_H

/// @file RobotProgramJsonIo.h
/// @brief 机器人程序 JSON

#include "cloudsim_host_global.h"

#include <QJsonArray>

class RobotProgramStore;

namespace cloudsim::host
{
class IRobotUrdfImportContext;

/// 机器人程序 JSON
CLOUDSIM_HOST_EXPORT QJsonArray robotProgramsToJson(const RobotProgramStore& store);
CLOUDSIM_HOST_EXPORT bool robotProgramsFromJson(RobotProgramStore& store, const QJsonArray& programs,
												IRobotUrdfImportContext& ctx, QString* outError = nullptr);

} // namespace cloudsim::host

#endif // CLOUDSIMHOST_ROBOTPROGRAMJSONIO_H
