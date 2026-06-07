// IOpParamConfig 通用实现：绑定 resource JSON schema
#pragma once

#include "IOpParamConfig.h"
#include "TrajectoryPipelineTypes.h"

#include <memory>
#include <string>

namespace trajectory_algo
{

class TrajectoryOpConfigImpl final : public IOpParamConfig
{
public:
	TrajectoryOpConfigImpl(
		RobotInstruction::TrajectoryOpKind kind,
		std::string jsonRelativePath);

	RobotInstruction::TrajectoryOpKind kind() const override { return m_kind; }
	std::string jsonRelativePath() const override { return m_jsonRelativePath; }
	std::vector<TrajectoryOpParamField> paramFields() const override;
	RobotInstruction::TrajectoryOpDescriptor defaultDescriptor(
		const RobotInstruction::OpScope& scope) const override;

private:
	RobotInstruction::TrajectoryOpKind m_kind;
	std::string m_jsonRelativePath;
};

std::unique_ptr<IOpParamConfig> makeTrajectoryOpConfig(
	RobotInstruction::TrajectoryOpKind kind,
	const char* jsonRelativePath);

} // namespace trajectory_algo
