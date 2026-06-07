// Delete 块参数字段与 descriptor 读写
#include "DeleteOpParamAccess.h"

namespace trajectory_algo
{

bool DeleteOpParamAccess::handlesKey(const std::string& key) const
{
	(void)key;
	return false;
}

bool DeleteOpParamAccess::read(
	const RobotInstruction::TrajectoryOpDescriptor& op,
	const TrajectoryOpParamField& field,
	TrajectoryParamValue& out) const
{
	(void)op;
	(void)field;
	(void)out;
	return false;
}

bool DeleteOpParamAccess::write(
	RobotInstruction::TrajectoryOpDescriptor& op,
	const TrajectoryOpParamField& field,
	const TrajectoryParamValue& in) const
{
	(void)op;
	(void)field;
	(void)in;
	return false;
}

std::unique_ptr<IOpParamAccess> makeDeleteOpParamAccess()
{
	return std::make_unique<DeleteOpParamAccess>();
}

} // namespace trajectory_algo
