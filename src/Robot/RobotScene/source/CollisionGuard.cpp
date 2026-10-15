/// @file CollisionGuard.cpp
/// @brief 碰撞联锁

#include "CollisionGuard.h"

CollisionGuard::Result CollisionGuard::evaluate(std::int64_t tickIndex) const
{
	Result r;
	if (!m_enabled || m_policy == CollisionGuardPolicy::Ignore || !m_probe)
	{
		return r;
	}
	if (m_everyN > 1 && (tickIndex % m_everyN) != 0)
	{
		return r;
	}
	r.hits = m_probe();
	if (r.hits.isEmpty())
	{
		return r;
	}
	if (m_policy == CollisionGuardPolicy::StopAll)
	{
		r.action = Action::StopAll;
		return r;
	}
	r.action = Action::FreezeLanes;
	for (const CollisionGuardHit& h : r.hits)
	{
		r.freezeIds.push_back(h.a);
		r.freezeIds.push_back(h.b);
	}
	return r;
}
