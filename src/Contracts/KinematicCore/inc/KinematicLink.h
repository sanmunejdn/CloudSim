#ifndef KINEMATICCORE_KINEMATICLINK_H
#define KINEMATICCORE_KINEMATICLINK_H

#include "kinematic_core_global.h"

#include <string>

namespace kinematic_core
{
struct KINEMATIC_CORE_API KinematicLink
{
	std::string id;
	std::string payloadKey;
	bool fixed = false;
	double restInBase[16]{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
};

} // namespace kinematic_core

#endif // KINEMATICCORE_KINEMATICLINK_H
