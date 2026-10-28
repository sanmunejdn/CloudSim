/// @file IkSolveMetrics.cpp
/// @brief IK 进程内原子计数

#include "IkSolveMetrics.h"

#include <atomic>
#include <sstream>

namespace RobotInstruction
{
namespace
{
std::atomic<std::uint64_t> g_success{0};
std::atomic<std::uint64_t> g_fail{0};
std::atomic<std::uint64_t> g_failBudget{0};
std::atomic<std::uint64_t> g_failWrist{0};
std::atomic<std::uint64_t> g_failAxisClass{0};
std::atomic<std::uint64_t> g_failOther{0};
std::atomic<std::uint64_t> g_totalElapsedMs{0};
std::atomic<std::uint64_t> g_samples{0};

void addElapsed(const int elapsedMs)
{
	if (elapsedMs > 0)
	{
		g_totalElapsedMs.fetch_add(static_cast<std::uint64_t>(elapsedMs), std::memory_order_relaxed);
	}
	g_samples.fetch_add(1, std::memory_order_relaxed);
}
} // namespace

void recordIkSolveSuccess(const int elapsedMs)
{
	g_success.fetch_add(1, std::memory_order_relaxed);
	addElapsed(elapsedMs);
}

void recordIkSolveFailure(const IkFailKind kind, const int elapsedMs)
{
	g_fail.fetch_add(1, std::memory_order_relaxed);
	switch (kind)
	{
	case IkFailKind::Budget:
		g_failBudget.fetch_add(1, std::memory_order_relaxed);
		break;
	case IkFailKind::Wrist:
		g_failWrist.fetch_add(1, std::memory_order_relaxed);
		break;
	case IkFailKind::AxisClass:
		g_failAxisClass.fetch_add(1, std::memory_order_relaxed);
		break;
	default:
		g_failOther.fetch_add(1, std::memory_order_relaxed);
		break;
	}
	addElapsed(elapsedMs);
}

IkSolveMetricsSnapshot ikSolveMetricsSnapshot()
{
	IkSolveMetricsSnapshot s;
	s.success = g_success.load(std::memory_order_relaxed);
	s.fail = g_fail.load(std::memory_order_relaxed);
	s.failBudget = g_failBudget.load(std::memory_order_relaxed);
	s.failWrist = g_failWrist.load(std::memory_order_relaxed);
	s.failAxisClass = g_failAxisClass.load(std::memory_order_relaxed);
	s.failOther = g_failOther.load(std::memory_order_relaxed);
	s.totalElapsedMs = g_totalElapsedMs.load(std::memory_order_relaxed);
	s.samples = g_samples.load(std::memory_order_relaxed);
	return s;
}

void resetIkSolveMetrics()
{
	g_success.store(0, std::memory_order_relaxed);
	g_fail.store(0, std::memory_order_relaxed);
	g_failBudget.store(0, std::memory_order_relaxed);
	g_failWrist.store(0, std::memory_order_relaxed);
	g_failAxisClass.store(0, std::memory_order_relaxed);
	g_failOther.store(0, std::memory_order_relaxed);
	g_totalElapsedMs.store(0, std::memory_order_relaxed);
	g_samples.store(0, std::memory_order_relaxed);
}

std::string formatIkSolveMetricsSummary()
{
	const IkSolveMetricsSnapshot s = ikSolveMetricsSnapshot();
	std::ostringstream os;
	os << "[ikStats ok=" << s.success << " fail=" << s.fail << " budget=" << s.failBudget << " wrist=" << s.failWrist
	   << " class=" << s.failAxisClass;
	if (s.samples > 0)
	{
		os << " avgMs=" << (s.totalElapsedMs / s.samples);
	}
	os << "]";
	return os.str();
}

} // namespace RobotInstruction
