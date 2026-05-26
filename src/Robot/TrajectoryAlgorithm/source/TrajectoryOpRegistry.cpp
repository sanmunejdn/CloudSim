#include "TrajectoryOpRegistry.h"

namespace trajectory_algo
{

TrajectoryOpRegistry& TrajectoryOpRegistry::instance()
{
	static TrajectoryOpRegistry registry;
	return registry;
}

void TrajectoryOpRegistry::registerOp(std::unique_ptr<ITrajectoryOp> op)
{
	if (!op)
	{
		return;
	}
	m_ops.push_back(std::move(op));
}

const ITrajectoryOp* TrajectoryOpRegistry::get(const RobotInstruction::TrajectoryOpKind kind) const
{
	for (const std::unique_ptr<ITrajectoryOp>& op : m_ops)
	{
		if (op && op->kind() == kind)
		{
			return op.get();
		}
	}
	return nullptr;
}

std::vector<RobotInstruction::TrajectoryOpKind> TrajectoryOpRegistry::paletteKinds() const
{
	std::vector<RobotInstruction::TrajectoryOpKind> kinds;
	kinds.reserve(m_ops.size());
	for (const std::unique_ptr<ITrajectoryOp>& op : m_ops)
	{
		if (op)
		{
			kinds.push_back(op->kind());
		}
	}
	return kinds;
}

} // namespace trajectory_algo
