/// @file TrajectoryOpRegistry.cpp
/// @brief TrajectoryOp 注册表

#include "TrajectoryOpRegistry.h"

#include <cassert>
#include <functional>
#include <mutex>
#include <vector>

namespace
{
trajectory_algo::TrajectoryOpRegistry* g_processTrajectoryOpRegistry = nullptr;
std::mutex g_deferredTrajectoryOpMutex;
std::vector<std::function<std::unique_ptr<trajectory_algo::ITrajectoryOp>()>> g_deferredTrajectoryOps;

void flushDeferredTrajectoryOps(trajectory_algo::TrajectoryOpRegistry& registry)
{
	std::vector<std::function<std::unique_ptr<trajectory_algo::ITrajectoryOp>()>> pending;
	{
		std::lock_guard<std::mutex> lock(g_deferredTrajectoryOpMutex);
		pending.swap(g_deferredTrajectoryOps);
	}
	for (const auto& factory : pending)
	{
		registry.registerOp(factory());
	}
}
} // namespace

namespace trajectory_algo
{
void deferTrajectoryOpRegistration(std::function<std::unique_ptr<ITrajectoryOp>()> factory)
{
	std::lock_guard<std::mutex> lock(g_deferredTrajectoryOpMutex);
	g_deferredTrajectoryOps.push_back(std::move(factory));
}

void TrajectoryOpRegistry::setProcessInstance(TrajectoryOpRegistry* registry)
{
	assert(registry != nullptr);
	g_processTrajectoryOpRegistry = registry;
	flushDeferredTrajectoryOps(*registry);
}

TrajectoryOpRegistry& TrajectoryOpRegistry::instance()
{
	assert(g_processTrajectoryOpRegistry != nullptr);
	return *g_processTrajectoryOpRegistry;
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
