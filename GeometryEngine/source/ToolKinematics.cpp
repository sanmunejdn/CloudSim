#include "ToolKinematics.h"

namespace engine
{

RigidTransform toolOriginFromFlange(const RigidTransform& baseFlange, const RigidTransform& flangeTool)
{
	// Eigen isometry product: p_base = R_flange * t_tool + t_flange (tool offset in flange axes).
	// Do not use composeScene here: URDF linkWorld (OSG) and tool (Eigen) adapters disagree on row-chain order.
	return baseFlange.composeColumn(flangeTool);
}

RigidTransform flangeFromToolOrigin(const RigidTransform& baseToolOrigin, const RigidTransform& flangeTool)
{
	return baseToolOrigin.composeColumn(flangeTool.inverse());
}

} // namespace engine
