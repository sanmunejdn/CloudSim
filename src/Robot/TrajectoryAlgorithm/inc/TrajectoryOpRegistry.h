#pragma once

#include "ITrajectoryOp.h"
#include "trajectory_algorithm_global.h"

#include <memory>
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

	TrajectoryOpRegistry(const TrajectoryOpRegistry&) = delete;
	TrajectoryOpRegistry& operator=(const TrajectoryOpRegistry&) = delete;
	TrajectoryOpRegistry(TrajectoryOpRegistry&&) = delete;
	TrajectoryOpRegistry& operator=(TrajectoryOpRegistry&&) = delete;
	~TrajectoryOpRegistry() = default;

private:
	TrajectoryOpRegistry() = default;
	std::vector<std::unique_ptr<ITrajectoryOp>> m_ops;
};

TRAJECTORY_ALGORITHM_API void ensureTrajectoryOpBuiltinsRegistered();

} // namespace trajectory_algo
