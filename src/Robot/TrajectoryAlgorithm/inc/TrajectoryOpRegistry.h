#ifndef TRAJECTORYALGORITHM_TRAJECTORYOPREGISTRY_H
#define TRAJECTORYALGORITHM_TRAJECTORYOPREGISTRY_H

/// @file TrajectoryOpRegistry.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief TrajectoryOpRegistry 接口

#include "trajectory_algorithm_global.h"

#include "ITrajectoryOp.h"

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace trajectory_algo
{
class TRAJECTORY_ALGORITHM_API TrajectoryOpRegistry
{
public:
	// 公开构造供 ServiceRegistry 持有；全局唯一性不再由类强制
	TrajectoryOpRegistry() = default;

	/// 宿主注册 ServiceRegistry 时写入，使本 DLL instance() 与宿主同一对象
	static void setProcessInstance(TrajectoryOpRegistry* registry);

	/// 业务路径须先 setProcessInstance（组合根）
	static TrajectoryOpRegistry& instance();

	void registerOp(std::unique_ptr<ITrajectoryOp> op);
	const ITrajectoryOp* get(RobotInstruction::TrajectoryOpKind kind) const;
	std::vector<RobotInstruction::TrajectoryOpKind> paletteKinds() const;

	std::string kindToString(RobotInstruction::TrajectoryOpKind kind) const;
	bool kindFromString(const std::string& token, RobotInstruction::TrajectoryOpKind& out) const;

	TrajectoryOpRegistry(const TrajectoryOpRegistry&) = delete;
	TrajectoryOpRegistry& operator=(const TrajectoryOpRegistry&) = delete;
	TrajectoryOpRegistry(TrajectoryOpRegistry&&) = delete;
	TrajectoryOpRegistry& operator=(TrajectoryOpRegistry&&) = delete;
	~TrajectoryOpRegistry() = default;

private:
	std::vector<std::unique_ptr<ITrajectoryOp>> m_ops;
	std::unordered_map<RobotInstruction::TrajectoryOpKind, std::string> m_kindToToken;
	std::unordered_map<std::string, RobotInstruction::TrajectoryOpKind> m_tokenToKind;
};

TRAJECTORY_ALGORITHM_API void ensureTrajectoryOpBuiltinsRegistered();

#define REGISTER_TRAJECTORY_OP(OpType)                                                                                \
	static const bool OpType##_registered = []()                                                                      \
	{                                                                                                                 \
		trajectory_algo::deferTrajectoryOpRegistration([]() { return std::make_unique<OpType>(); });                  \
		return true;                                                                                                    \
	}()

/// 静态注册宏用：进程槽就绪前只入队，由 setProcessInstance / ensure 刷入
TRAJECTORY_ALGORITHM_API void deferTrajectoryOpRegistration(
	std::function<std::unique_ptr<ITrajectoryOp>()> factory);

} // namespace trajectory_algo

#endif // TRAJECTORYALGORITHM_TRAJECTORYOPREGISTRY_H
