#pragma once

#include "CoreTypes.h"
#include "cloudsim_host_global.h"

#include <json.hpp>

#include <string>
#include <vector>

namespace cloudsim::host {

CLOUDSIM_HOST_EXPORT QVector<core::PropertyRowDto> propertyRowsFromInstructionSnapshotJson(const nlohmann::json& rows);

CLOUDSIM_HOST_EXPORT core::FeasibleMotionAxisOptionsDto feasibleAxisOptionsFromEngine(
	const std::vector<std::string>& preset,
	const std::vector<std::string>& elbow,
	const std::vector<std::string>& wrist,
	const std::vector<std::string>& arm,
	const std::vector<std::string>& turnJ1,
	const std::vector<std::string>& turnJ4,
	const std::vector<std::string>& turnJ6);

} // namespace cloudsim::host
