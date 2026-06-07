// Reorder 块参数字段与 descriptor 读写
#include "ReorderOpParamAccess.h"

namespace trajectory_algo
{

bool ReorderOpParamAccess::handlesKey(const std::string& key) const
{
	(void)key;
	return false;
}

bool ReorderOpParamAccess::read(
	const RobotInstruction::TrajectoryOpDescriptor& op,
	const TrajectoryOpParamField& field,
	TrajectoryParamValue& out) const
{
	(void)op;
	(void)field;
	(void)out;
	return false;
}

bool ReorderOpParamAccess::write(
	RobotInstruction::TrajectoryOpDescriptor& op,
	const TrajectoryOpParamField& field,
	const TrajectoryParamValue& in) const
{
	(void)op;
	(void)field;
	(void)in;
	return false;
}

std::unique_ptr<IOpParamAccess> makeReorderOpParamAccess()
{
	return std::make_unique<ReorderOpParamAccess>();
}

} // namespace trajectory_algo
