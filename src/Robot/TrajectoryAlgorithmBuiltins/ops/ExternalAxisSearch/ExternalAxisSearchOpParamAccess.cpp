// ExternalAxisSearch 块参数字段与 descriptor 读写
#include "ExternalAxisSearchOpParamAccess.h"

#include "TrajectoryPipelineTypes.h"

namespace trajectory_algo
{

bool ExternalAxisSearchOpParamAccess::handlesKey(const std::string& key) const
{
	return false;
}

bool ExternalAxisSearchOpParamAccess::read(
	const RobotInstruction::TrajectoryOpDescriptor& op,
	const TrajectoryOpParamField& field,
	TrajectoryParamValue& out) const
{
	(void)op;
	(void)field;
	(void)out;
	return false;
}

bool ExternalAxisSearchOpParamAccess::write(
	RobotInstruction::TrajectoryOpDescriptor& op,
	const TrajectoryOpParamField& field,
	const TrajectoryParamValue& in) const
{
	(void)op;
	(void)field;
	(void)in;
	return false;
}

std::unique_ptr<IOpParamAccess> makeExternalAxisSearchOpParamAccess()
{
	return std::make_unique<ExternalAxisSearchOpParamAccess>();
}

} // namespace trajectory_algo
