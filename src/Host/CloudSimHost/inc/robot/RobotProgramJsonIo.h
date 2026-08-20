#ifndef CLOUDSIMHOST_ROBOTPROGRAMJSONIO_H
#define CLOUDSIMHOST_ROBOTPROGRAMJSONIO_H

/// @file RobotProgramJsonIo.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
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
