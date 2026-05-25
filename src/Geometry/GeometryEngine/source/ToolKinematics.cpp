#include "ToolKinematics.h"

namespace engine
{

RigidTransform toolOriginFromFlange(const RigidTransform& baseFlange, const RigidTransform& flangeTool)
{
	// Eigen 等距积：p_base=R_flange*t_tool+t_flange
	// 勿 composeScene：URDF OSG 与 Eigen 链序不一致
	return baseFlange.composeColumn(flangeTool);
}

RigidTransform flangeFromToolOrigin(const RigidTransform& baseToolOrigin, const RigidTransform& flangeTool)
{
	return baseToolOrigin.composeColumn(flangeTool.inverse());
}

} // namespace engine
