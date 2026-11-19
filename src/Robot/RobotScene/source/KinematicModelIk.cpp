/// @file KinematicModelIk.cpp
/// @brief KinematicModelIk 实现

#include "KinematicModelIk.h"

#include "IkSolveSession.h"

namespace KinematicModelIk
{
RobotTeachIk::TeachIkResult solveTeachPose(const std::string& registryKey, RobotTeachIk::TeachIkContext ctx)
{
	ctx.registryKey = registryKey;
	return RobotInstruction::runIkSolveSession(std::move(ctx));
}

} // namespace KinematicModelIk
