// ReachabilityFilter 块参数字段与 descriptor 读写
#include "ReachabilityFilterOpParamAccess.h"

#include "TrajectoryPipelineTypes.h"

namespace trajectory_algo
{

bool ReachabilityFilterOpParamAccess::handlesKey(const std::string& key) const
{
	return false;
}

bool ReachabilityFilterOpParamAccess::read(
	const RobotInstruction::TrajectoryOpDescriptor& op,
	const TrajectoryOpParamField& field,
	TrajectoryParamValue& out) const
{
	(void)op;
	(void)field;
	(void)out;
	return false;
}

bool ReachabilityFilterOpParamAccess::write(
	RobotInstruction::TrajectoryOpDescriptor& op,
	const TrajectoryOpParamField& field,
	const TrajectoryParamValue& in) const
{
	(void)op;
	(void)field;
	(void)in;
	return false;
}

std::unique_ptr<IOpParamAccess> makeReachabilityFilterOpParamAccess()
{
	return std::make_unique<ReachabilityFilterOpParamAccess>();
}

} // namespace trajectory_algo
