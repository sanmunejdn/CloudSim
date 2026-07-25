/// @file DispatchPolicies.cpp
/// @brief FIFO / SPT / LPT / EDD / CR

#include "sim/DispatchPolicies.h"

#include <QString>
#include <QStringList>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>

namespace
{
bool betterPriorityThenFifo(const ReadyOpCandidate& a, const ReadyOpCandidate& b)
{
	if (a.priority != b.priority)
	{
		return a.priority > b.priority;
	}
	return a.enqueueTime < b.enqueueTime;
}

int pickBy(const DispatchContext& ctx, const std::function<bool(const ReadyOpCandidate&, const ReadyOpCandidate&)>& better)
{
	if (ctx.candidates.isEmpty())
	{
		return -1;
	}
	int best = 0;
	for (int i = 1; i < ctx.candidates.size(); ++i)
	{
		if (better(ctx.candidates[i], ctx.candidates[best]))
		{
			best = i;
		}
	}
	return best;
}
} // namespace

int FifoPolicy::select(const DispatchContext& ctx) const
{
	return pickBy(ctx, betterPriorityThenFifo);
}

int SptPolicy::select(const DispatchContext& ctx) const
{
	return pickBy(ctx,
				  [](const ReadyOpCandidate& a, const ReadyOpCandidate& b)
				  {
					  if (a.processTimeSec < b.processTimeSec - 1e-12)
						  return true;
					  if (std::abs(a.processTimeSec - b.processTimeSec) <= 1e-12)
						  return betterPriorityThenFifo(a, b);
					  return false;
				  });
}

int LptPolicy::select(const DispatchContext& ctx) const
{
	return pickBy(ctx,
				  [](const ReadyOpCandidate& a, const ReadyOpCandidate& b)
				  {
					  if (a.processTimeSec > b.processTimeSec + 1e-12)
						  return true;
					  if (std::abs(a.processTimeSec - b.processTimeSec) <= 1e-12)
						  return betterPriorityThenFifo(a, b);
					  return false;
				  });
}

int EddPolicy::select(const DispatchContext& ctx) const
{
	return pickBy(ctx,
				  [](const ReadyOpCandidate& a, const ReadyOpCandidate& b)
				  {
					  if (a.dueDateSec < b.dueDateSec - 1e-12)
						  return true;
					  if (std::abs(a.dueDateSec - b.dueDateSec) <= 1e-12)
						  return betterPriorityThenFifo(a, b);
					  return false;
				  });
}

int CrPolicy::select(const DispatchContext& ctx) const
{
	return pickBy(ctx,
				  [&ctx](const ReadyOpCandidate& a, const ReadyOpCandidate& b)
				  {
					  const double ra = a.remainingWorkSec > 1e-9 ? (a.dueDateSec - ctx.now) / a.remainingWorkSec
																 : std::numeric_limits<double>::infinity();
					  const double rb = b.remainingWorkSec > 1e-9 ? (b.dueDateSec - ctx.now) / b.remainingWorkSec
																 : std::numeric_limits<double>::infinity();
					  if (ra < rb - 1e-12)
						  return true;
					  if (std::abs(ra - rb) <= 1e-12)
						  return betterPriorityThenFifo(a, b);
					  return false;
				  });
}

std::unique_ptr<IDispatchPolicy> createDispatchPolicy(const QString& name)
{
	const QString n = name.toLower();
	if (n == QStringLiteral("spt"))
		return std::make_unique<SptPolicy>();
	if (n == QStringLiteral("lpt"))
		return std::make_unique<LptPolicy>();
	if (n == QStringLiteral("edd"))
		return std::make_unique<EddPolicy>();
	if (n == QStringLiteral("cr"))
		return std::make_unique<CrPolicy>();
	return std::make_unique<FifoPolicy>();
}

QStringList allDispatchPolicyNames()
{
	return {QStringLiteral("fifo"), QStringLiteral("spt"), QStringLiteral("lpt"), QStringLiteral("edd"),
			QStringLiteral("cr")};
}
