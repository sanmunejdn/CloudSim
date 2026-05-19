#include "ToolKinematics.h"

namespace engine
{

RigidTransform toolOriginFromFlange(const RigidTransform& baseFlange, const RigidTransform& flangeTool)
{
	return baseFlange.composeScene(flangeTool);
}

RigidTransform flangeFromToolOrigin(const RigidTransform& baseToolOrigin, const RigidTransform& flangeTool)
{
	return baseToolOrigin.composeScene(flangeTool.inverse());
}

} // namespace engine
