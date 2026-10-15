#ifndef ROBOTSCENE_COLLISIONGUARD_H
#define ROBOTSCENE_COLLISIONGUARD_H

/// @file CollisionGuard.h
/// @brief 运行时碰撞联锁策略（检测由 UI/Host 注入）

#include "robot_scene_global.h"

#include "SimulationParticipantId.h"

#include <QVector>
#include <cstdint>
#include <functional>

enum class ROBOT_SCENE_API CollisionGuardPolicy
{
	Ignore = 0,
	FreezeLane,
	StopAll
};

struct ROBOT_SCENE_API CollisionGuardHit
{
	SimulationParticipantId a;
	SimulationParticipantId b;
};

class ROBOT_SCENE_API CollisionGuard
{
public:
	void setPolicy(CollisionGuardPolicy p) { m_policy = p; }
	CollisionGuardPolicy policy() const { return m_policy; }
	void setEnabled(bool on) { m_enabled = on; }
	bool enabled() const { return m_enabled; }
	void setCheckEveryNTicks(int n) { m_everyN = n < 1 ? 1 : n; }

	using ProbeFn = std::function<QVector<CollisionGuardHit>()>;
	void setProbe(ProbeFn probe) { m_probe = std::move(probe); }

	enum class Action
	{
		None,
		FreezeLanes,
		StopAll
	};

	struct Result
	{
		Action action = Action::None;
		QVector<SimulationParticipantId> freezeIds;
		QVector<CollisionGuardHit> hits;
	};

	Result evaluate(std::int64_t tickIndex) const;

private:
	bool m_enabled = false;
	CollisionGuardPolicy m_policy = CollisionGuardPolicy::FreezeLane;
	int m_everyN = 4;
	ProbeFn m_probe;
};

#endif // ROBOTSCENE_COLLISIONGUARD_H
