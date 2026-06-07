// 原子块参数字段读写接口，解耦 TrajectoryOpDescriptor 与 UI/校验
#pragma once

#include "TrajectoryOpParamSchema.h"
#include "TrajectoryPipelineTypes.h"
#include "trajectory_algorithm_global.h"

namespace trajectory_algo
{

class TRAJECTORY_ALGORITHM_API IOpParamAccess
{
public:
	virtual ~IOpParamAccess() = default;

	virtual bool handlesKey(const std::string& key) const = 0;
	virtual bool read(
		const RobotInstruction::TrajectoryOpDescriptor& op,
		const TrajectoryOpParamField& field,
		TrajectoryParamValue& out) const = 0;
	virtual bool write(
		RobotInstruction::TrajectoryOpDescriptor& op,
		const TrajectoryOpParamField& field,
		const TrajectoryParamValue& in) const = 0;
};

} // namespace trajectory_algo
