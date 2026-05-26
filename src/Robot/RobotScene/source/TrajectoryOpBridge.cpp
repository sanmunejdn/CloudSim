#include "TrajectoryOpBridge.h"

#include "TrajectoryOpDescriptorCodec.h"
#include "TrajectoryOpParamAccess.h"
#include "TrajectoryOpRegistry.h"

namespace RobotInstruction
{

trajectory_algo::TrajectoryOpRegistry& trajectoryOpRegistry()
{
	return trajectory_algo::TrajectoryOpRegistry::instance();
}

void ensureTrajectoryOpBuiltinsRegistered()
{
	trajectory_algo::ensureTrajectoryOpBuiltinsRegistered();
}

const trajectory_algo::ITrajectoryOp* trajectoryOpGet(const TrajectoryOpKind kind)
{
	return trajectory_algo::TrajectoryOpRegistry::instance().get(kind);
}

std::vector<TrajectoryOpKind> trajectoryOpPaletteKinds()
{
	return trajectory_algo::TrajectoryOpRegistry::instance().paletteKinds();
}

std::vector<trajectory_algo::TrajectoryOpParamField> trajectoryOpAllParamFields(
	const trajectory_algo::ITrajectoryOp& op)
{
	return trajectory_algo::TrajectoryOpParamAccess::allFieldsForOp(op);
}

bool trajectoryOpParamRead(
	const TrajectoryOpDescriptor& op,
	const trajectory_algo::TrajectoryOpParamField& field,
	trajectory_algo::TrajectoryParamValue& out)
{
	return trajectory_algo::TrajectoryOpParamAccess::read(op, field, out);
}

bool trajectoryOpParamWrite(
	TrajectoryOpDescriptor& op,
	const trajectory_algo::TrajectoryOpParamField& field,
	const trajectory_algo::TrajectoryParamValue& value)
{
	return trajectory_algo::TrajectoryOpParamAccess::write(op, field, value);
}

nlohmann::json trajectoryPipelineToJson(const std::vector<TrajectoryOpDescriptor>& ops)
{
	return trajectory_algo::pipelineToJson(ops);
}

bool trajectoryPipelineFromJson(
	const nlohmann::json& j,
	std::vector<TrajectoryOpDescriptor>& out,
	std::string* errMsg)
{
	return trajectory_algo::pipelineFromJson(j, out, errMsg);
}

} // namespace RobotInstruction
