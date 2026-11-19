/// @file IkSolveSession.cpp
/// @brief 外轴路径选择后委托 TeachIk

#include "IkSolveSession.h"

#include "UrdfIkMachineProfile.h"

namespace RobotInstruction
{
namespace
{
void applyExternalCoupleMode(RobotTeachIk::TeachIkContext& ctx)
{
	if (ctx.urdfPath.isEmpty())
	{
		return;
	}
	const UrdfRobotLoader::UrdfIkMachineProfile mach = UrdfRobotLoader::resolveIkMachineProfile(ctx.urdfPath);
	if (mach.externalCoupleMode == UrdfRobotLoader::ExternalCoupleMode::ArmOnly)
	{
		ctx.externalAxis = {};
		ctx.externalAxes = {};
		ctx.externalAxisConfigCount = 0;
		return;
	}
	if (mach.externalCoupleMode == UrdfRobotLoader::ExternalCoupleMode::GridOnly)
	{
		ctx.externalAxis.optimizeExternal = false;
		ctx.externalAxes.optimizeExternal = false;
	}
}
} // namespace

RobotTeachIk::TeachIkResult runIkSolveSession(RobotTeachIk::TeachIkContext ctx)
{
	applyExternalCoupleMode(ctx);
	return RobotTeachIk::solveTeachIk(ctx);
}

RobotTeachIk::TeachIkResult runIkSolveSessionCoordinatedDrag(RobotTeachIk::TeachIkContext ctx,
															 const double qExternalHintMm, const bool hasExternalHint)
{
	applyExternalCoupleMode(ctx);
	return RobotTeachIk::solveTeachIkCoordinatedDrag(ctx, qExternalHintMm, hasExternalHint);
}

} // namespace RobotInstruction
