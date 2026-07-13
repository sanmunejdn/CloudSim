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
	const RobotInstruction::TrajectoryOpKind kind = op->kind();
	const char* token = op->kindToken();
	if (token && token[0] != '\0')
	{
		m_kindToToken[kind] = token;
		m_tokenToKind[token] = kind;
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

std::string TrajectoryOpRegistry::kindToString(const RobotInstruction::TrajectoryOpKind kind) const
{
	const auto it = m_kindToToken.find(kind);
	if (it != m_kindToToken.end())
	{
		return it->second;
	}
	return "Translate";
}

bool TrajectoryOpRegistry::kindFromString(const std::string& token, RobotInstruction::TrajectoryOpKind& out) const
{
	const auto it = m_tokenToKind.find(token);
	if (it == m_tokenToKind.end())
	{
		return false;
	}
	out = it->second;
	return true;
}

} // namespace trajectory_algo
