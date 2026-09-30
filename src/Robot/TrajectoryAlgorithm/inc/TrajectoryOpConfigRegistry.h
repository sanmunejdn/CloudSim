#ifndef TRAJECTORYALGORITHM_TRAJECTORYOPCONFIGREGISTRY_H
#define TRAJECTORYALGORITHM_TRAJECTORYOPCONFIGREGISTRY_H

/// @file TrajectoryOpConfigRegistry.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief TrajectoryOpConfigRegistry 接口

// schema 进程一份（组合根注入）；resourceBaseDir 为线程加载态，由 ensureLoaded 写入 TLS
#include "trajectory_algorithm_global.h"

#include "IOpParamConfig.h"
#include "TrajectoryOpParamSchema.h"
#include "TrajectoryPipelineTypes.h"

#include <memory>
#include <string>
#include <vector>

namespace trajectory_algo
{
class TRAJECTORY_ALGORITHM_API TrajectoryOpConfigRegistry
{
public:
	// 公开构造供 ServiceRegistry 持有
	TrajectoryOpConfigRegistry() = default;

	static void setProcessInstance(TrajectoryOpConfigRegistry* registry);
	/// 业务路径须先 setProcessInstance（组合根）
	static TrajectoryOpConfigRegistry& instance();

	void registerOpConfig(std::unique_ptr<IOpParamConfig> config);

	/// 写入当前线程加载态目录（多文档应在操作前用该文档的 baseDir 调用）
	bool ensureLoaded(const std::string& resourceBaseDir, std::string* errMsg = nullptr);
	const std::string& resourceBaseDir() const;

	std::vector<TrajectoryOpParamField> paramFieldsForOp(RobotInstruction::TrajectoryOpKind kind) const;
	RobotInstruction::TrajectoryOpDescriptor defaultUnifiedOp(RobotInstruction::TrajectoryOpKind kind,
															  const RobotInstruction::OpScope& scope) const;

	TrajectoryOpConfigRegistry(const TrajectoryOpConfigRegistry&) = delete;
	TrajectoryOpConfigRegistry& operator=(const TrajectoryOpConfigRegistry&) = delete;
	TrajectoryOpConfigRegistry(TrajectoryOpConfigRegistry&&) = delete;
	TrajectoryOpConfigRegistry& operator=(TrajectoryOpConfigRegistry&&) = delete;
	~TrajectoryOpConfigRegistry() = default;

private:
	const IOpParamConfig* configFor(const RobotInstruction::TrajectoryOpKind kind) const;

	std::vector<std::unique_ptr<IOpParamConfig>> m_configs;
};

} // namespace trajectory_algo

#endif // TRAJECTORYALGORITHM_TRAJECTORYOPCONFIGREGISTRY_H
