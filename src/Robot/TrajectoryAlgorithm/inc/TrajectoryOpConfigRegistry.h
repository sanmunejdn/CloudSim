// 集中注册各块 schema，懒加载 trajectory 资源目录
#pragma once

#include "IOpParamConfig.h"
#include "TrajectoryOpParamSchema.h"
#include "TrajectoryPipelineTypes.h"
#include "trajectory_algorithm_global.h"

#include <memory>
#include <string>
#include <vector>

namespace trajectory_algo
{

class TRAJECTORY_ALGORITHM_API TrajectoryOpConfigRegistry
{
public:
	static TrajectoryOpConfigRegistry& instance();

	void registerOpConfig(std::unique_ptr<IOpParamConfig> config);

	bool ensureLoaded(const std::string& resourceBaseDir, std::string* errMsg = nullptr);
	const std::string& resourceBaseDir() const { return m_resourceBaseDir; }

	std::vector<TrajectoryOpParamField> paramFieldsForOp(RobotInstruction::TrajectoryOpKind kind) const;
	RobotInstruction::TrajectoryOpDescriptor defaultUnifiedOp(
		RobotInstruction::TrajectoryOpKind kind,
		const RobotInstruction::OpScope& scope) const;

	TrajectoryOpConfigRegistry(const TrajectoryOpConfigRegistry&) = delete;
	TrajectoryOpConfigRegistry& operator=(const TrajectoryOpConfigRegistry&) = delete;
	TrajectoryOpConfigRegistry(TrajectoryOpConfigRegistry&&) = delete;
	TrajectoryOpConfigRegistry& operator=(TrajectoryOpConfigRegistry&&) = delete;
	~TrajectoryOpConfigRegistry() = default;

private:
	TrajectoryOpConfigRegistry() = default;

	const IOpParamConfig* configFor(const RobotInstruction::TrajectoryOpKind kind) const;

	std::string m_resourceBaseDir;
	bool m_loaded = false;
	std::vector<std::unique_ptr<IOpParamConfig>> m_configs;
};

} // namespace trajectory_algo
