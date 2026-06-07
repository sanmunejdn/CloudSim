// 原子块参数 schema 与默认描述符接口，对应 resource/trajectory JSON
#pragma once

#include "TrajectoryOpParamSchema.h"
#include "TrajectoryPipelineTypes.h"
#include "trajectory_algorithm_global.h"

#include <string>
#include <vector>

namespace trajectory_algo
{

class TRAJECTORY_ALGORITHM_API IOpParamConfig
{
public:
	virtual ~IOpParamConfig() = default;

	virtual RobotInstruction::TrajectoryOpKind kind() const = 0;
	virtual std::string jsonRelativePath() const = 0;
	virtual std::vector<TrajectoryOpParamField> paramFields() const = 0;
	virtual RobotInstruction::TrajectoryOpDescriptor defaultDescriptor(
		const RobotInstruction::OpScope& scope) const = 0;
};

} // namespace trajectory_algo
