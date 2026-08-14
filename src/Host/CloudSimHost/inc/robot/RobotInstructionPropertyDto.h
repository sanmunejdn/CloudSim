#ifndef CLOUDSIMHOST_ROBOTINSTRUCTIONPROPERTYDTO_H
#define CLOUDSIMHOST_ROBOTINSTRUCTIONPROPERTYDTO_H

/// @file RobotInstructionPropertyDto.h
/// @brief RobotInstructionPropertyDto 接口

#include "cloudsim_host_global.h"

#include "CoreTypes.h"

#include <string>
#include <vector>

#include <json.hpp>

namespace cloudsim::host
{
CLOUDSIM_HOST_EXPORT QVector<core::PropertyRowDto> propertyRowsFromInstructionSnapshotJson(const nlohmann::json& rows);

CLOUDSIM_HOST_EXPORT core::FeasibleMotionAxisOptionsDto
feasibleAxisOptionsFromEngine(const std::vector<std::string>& preset, const std::vector<std::string>& elbow,
							  const std::vector<std::string>& wrist, const std::vector<std::string>& arm,
							  const std::vector<std::string>& turnJ1, const std::vector<std::string>& turnJ4,
							  const std::vector<std::string>& turnJ6);

} // namespace cloudsim::host

#endif // CLOUDSIMHOST_ROBOTINSTRUCTIONPROPERTYDTO_H
