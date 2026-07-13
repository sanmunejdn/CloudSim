#pragma once

#include "ITrajectoryOp.h"
#include "trajectory_algorithm_global.h"

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace trajectory_algo
{

class TRAJECTORY_ALGORITHM_API TrajectoryOpRegistry
{
public:
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
	TrajectoryOpRegistry() = default;
	std::vector<std::unique_ptr<ITrajectoryOp>> m_ops;
	std::unordered_map<RobotInstruction::TrajectoryOpKind, std::string> m_kindToToken;
	std::unordered_map<std::string, RobotInstruction::TrajectoryOpKind> m_tokenToKind;
};

TRAJECTORY_ALGORITHM_API void ensureTrajectoryOpBuiltinsRegistered();

#define REGISTER_TRAJECTORY_OP(OpType) \
	static const bool OpType##_registered = []() { \
		trajectory_algo::TrajectoryOpRegistry::instance().registerOp(std::make_unique<OpType>()); \
		return true; \
	}()

} // namespace trajectory_algo
