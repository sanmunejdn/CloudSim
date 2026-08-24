#include "KinematicModelIk.h"

namespace KinematicModelIk
{
RobotTeachIk::TeachIkResult solveTeachPose(const std::string& registryKey, RobotTeachIk::TeachIkContext ctx)
{
	ctx.registryKey = registryKey;
	return RobotTeachIk::solveTeachIk(ctx);
}

} // namespace KinematicModelIk
