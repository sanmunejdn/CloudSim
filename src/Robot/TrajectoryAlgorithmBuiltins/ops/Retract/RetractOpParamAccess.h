// Retract 块参数字段与 descriptor 读写
#pragma once

#include "IOpParamAccess.h"

#include <memory>

namespace trajectory_algo
{

class RetractOpParamAccess final : public IOpParamAccess
{
public:
	bool handlesKey(const std::string& key) const override;
	bool read(
		const RobotInstruction::TrajectoryOpDescriptor& op,
		const TrajectoryOpParamField& field,
		TrajectoryParamValue& out) const override;
	bool write(
		RobotInstruction::TrajectoryOpDescriptor& op,
		const TrajectoryOpParamField& field,
		const TrajectoryParamValue& in) const override;
};

std::unique_ptr<IOpParamAccess> makeRetractOpParamAccess();

} // namespace trajectory_algo
