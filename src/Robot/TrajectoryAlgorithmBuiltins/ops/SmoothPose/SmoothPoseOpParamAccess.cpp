// SmoothPose 块参数字段与 descriptor 读写
#include "SmoothPoseOpParamAccess.h"

#include "TrajectoryPipelineTypes.h"

namespace trajectory_algo
{

bool SmoothPoseOpParamAccess::handlesKey(const std::string& key) const
{
	return false;
}

bool SmoothPoseOpParamAccess::read(
	const RobotInstruction::TrajectoryOpDescriptor& op,
	const TrajectoryOpParamField& field,
	TrajectoryParamValue& out) const
{
	(void)op;
	(void)field;
	(void)out;
	return false;
}

bool SmoothPoseOpParamAccess::write(
	RobotInstruction::TrajectoryOpDescriptor& op,
	const TrajectoryOpParamField& field,
	const TrajectoryParamValue& in) const
{
	(void)op;
	(void)field;
	(void)in;
	return false;
}

std::unique_ptr<IOpParamAccess> makeSmoothPoseOpParamAccess()
{
	return std::make_unique<SmoothPoseOpParamAccess>();
}

} // namespace trajectory_algo
