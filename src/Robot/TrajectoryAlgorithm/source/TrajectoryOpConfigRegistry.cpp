/// @file TrajectoryOpConfigRegistry.cpp
/// @brief TrajectoryOpConfig 注册表

// TrajectoryOpConfigRegistry 实现
#include "TrajectoryOpConfigRegistry.h"

#include "ITrajectoryOp.h"
#include "TrajectoryOpRegistry.h"
#include "TrajectoryParamJsonIo.h"

namespace trajectory_algo
{
TrajectoryOpConfigRegistry& TrajectoryOpConfigRegistry::instance()
{
	static TrajectoryOpConfigRegistry registry;
	return registry;
}

void TrajectoryOpConfigRegistry::registerOpConfig(std::unique_ptr<IOpParamConfig> config)
{
	if (!config)
	{
		return;
	}
	m_configs.push_back(std::move(config));
}

bool TrajectoryOpConfigRegistry::ensureLoaded(const std::string& resourceBaseDir, std::string* errMsg)
{
	(void)errMsg;
	if (!resourceBaseDir.empty())
	{
		m_resourceBaseDir = resourceBaseDir;
	}
	m_loaded = true;
	return true;
}

const IOpParamConfig* TrajectoryOpConfigRegistry::configFor(const RobotInstruction::TrajectoryOpKind kind) const
{
	for (const std::unique_ptr<IOpParamConfig>& config : m_configs)
	{
		if (config && config->kind() == kind)
		{
			return config.get();
		}
	}
	return nullptr;
}

std::vector<TrajectoryOpParamField>
TrajectoryOpConfigRegistry::paramFieldsForOp(const RobotInstruction::TrajectoryOpKind kind) const
{
	std::vector<TrajectoryOpParamField> fields = loadCommonScopeFieldsFromJson(m_resourceBaseDir);
	const IOpParamConfig* config = configFor(kind);
	if (config)
	{
		const std::vector<TrajectoryOpParamField> algoFields = config->paramFields();
		fields.insert(fields.end(), algoFields.begin(), algoFields.end());
		return fields;
	}
	const ITrajectoryOp* algo = TrajectoryOpRegistry::instance().get(kind);
	if (algo)
	{
		const std::vector<TrajectoryOpParamField> algoFields = algo->paramFields();
		fields.insert(fields.end(), algoFields.begin(), algoFields.end());
	}
	return fields;
}

RobotInstruction::TrajectoryOpDescriptor
TrajectoryOpConfigRegistry::defaultUnifiedOp(const RobotInstruction::TrajectoryOpKind kind,
											 const RobotInstruction::OpScope& scope) const
{
	const IOpParamConfig* config = configFor(kind);
	if (config)
	{
		return config->defaultDescriptor(scope);
	}
	const ITrajectoryOp* algo = TrajectoryOpRegistry::instance().get(kind);
	if (algo)
	{
		return algo->makeDefaultDescriptor(scope);
	}
	RobotInstruction::TrajectoryOpDescriptor op{};
	op.kind = kind;
	op.scope = scope;
	return op;
}

} // namespace trajectory_algo
